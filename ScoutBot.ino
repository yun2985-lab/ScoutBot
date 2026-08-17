/*
 * ScoutBot — DIY AI scout robot on LilyGo T-Camera S3 (ESP32-S3)
 *
 * Captures photos on motion detection or Telegram command, describes them
 * with a local vision-language model (Ollama on LAN), reports over Telegram,
 * and archives everything to a PC for later fine-tuning.
 *
 * https://github.com/yun2985-lab/ScoutBot
 *
 * ---------------------------------------------------------------------------
 * SETUP: fill in the CONFIG section below before flashing.
 *
 * Arduino IDE settings (all required):
 *   Board             : ESP32S3 Dev Module
 *   PSRAM             : OPI PSRAM        <- camera init fails without this
 *   USB CDC On Boot   : Enabled          <- no serial output without this
 *   Partition Scheme  : Huge APP (3MB No OTA)
 *   Flash Size        : 16MB
 *   ESP32 core        : 2.0.17
 * ---------------------------------------------------------------------------
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "img_converters.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "esp_http_server.h"

#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

// Enums live in a separate header on purpose: the Arduino IDE injects
// auto-generated function prototypes at the top of the .ino, which would
// otherwise reference these types before they are declared.
#include "face_types.h"

// ===========================================================================
// CONFIG — fill these in
// ===========================================================================

const char* ssid     = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// PC running Ollama and the Flask archive server (same LAN)
const char* ollama_url  = "http://192.168.0.10:11434/api/chat";
const char* storage_url = "http://192.168.0.10:5005/upload";

// Vision model tag as registered in Ollama
#define VLM_MODEL "your-vision-model:tag"

// Telegram bot — get a token from @BotFather
const char* telegram_token   = "YOUR_BOT_TOKEN";
const char* telegram_chat_id = "YOUR_CHAT_ID";   // system alerts go here

// Users allowed to issue commands.
// NOTE: Telegram user IDs can exceed 32 bits — long long is required.
const long long ALLOWED_USERS[] = {
  100000000LL,   // owner
  200000000LL    // second user
};
const int ALLOWED_USER_COUNT = sizeof(ALLOWED_USERS) / sizeof(ALLOWED_USERS[0]);

// Who receives captured photos
const char* PHOTO_RECIPIENTS[] = {
  "100000000",
  "200000000"
};
const int PHOTO_RECIPIENT_COUNT = sizeof(PHOTO_RECIPIENTS) / sizeof(PHOTO_RECIPIENTS[0]);

// Command phrases that trigger a capture
#define CMD_PHOTO_1 "/photo"
#define CMD_PHOTO_2 "take a photo"

// ===========================================================================
// Feature flags
// ===========================================================================

#define PIR_ENABLED             false   // PIR trigger (unused; kept for reference)
#define BRIGHTNESS_CHECK        true    // skip captures that are too dark
#define BRIGHTNESS_THRESHOLD    45      // 0-255; tune against your room

#define MOTION_DETECT_ENABLED   true
#define MOTION_THRESHOLD        40      // mean pixel delta; lower = more sensitive
#define MOTION_CHECK_INTERVAL   2500    // ms between checks
#define MOTION_COOLDOWN         60000   // ms before another auto-capture
#define MOTION_SEND_TELEGRAM    false   // auto-captures archive only, no push

// ===========================================================================
// Resolutions
// ===========================================================================

#define STREAM_FRAMESIZE   FRAMESIZE_VGA    // MJPEG stream
#define CAPTURE_FRAMESIZE  FRAMESIZE_SXGA   // actual photo
#define WARMUP_FRAMES      3                // discard after switching size

// ===========================================================================
// Pin map — LilyGo T-Camera S3 V1.6
// ===========================================================================

#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  39
#define XCLK_GPIO_NUM   38
#define SIOD_GPIO_NUM   5
#define SIOC_GPIO_NUM   4
#define VSYNC_GPIO_NUM  8
#define HREF_GPIO_NUM   18
#define PCLK_GPIO_NUM   12
#define Y9_GPIO_NUM     9
#define Y8_GPIO_NUM     10
#define Y7_GPIO_NUM     11
#define Y6_GPIO_NUM     13
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     48
#define Y3_GPIO_NUM     47
#define Y2_GPIO_NUM     14

#define I2C_SDA 7          // shared by OLED and PMU
#define I2C_SCL 6
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define PIR_PIN 17

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
XPowersPMU PMU;

bool isAllowedUser(long long userId) {
  for (int i = 0; i < ALLOWED_USER_COUNT; i++) {
    if (ALLOWED_USERS[i] == userId) return true;
  }
  return false;
}

// ===========================================================================
// Animated face
//
// The OLED shows a pair of eyes rather than status text. Left and right eye
// heights are tracked separately so the robot can wink.
// ===========================================================================

FaceMood currentMood = FACE_IDLE;

const int EYE_W = 28;
const int EYE_H = 28;
const int EYE_GAP = 14;
const int FACE_CENTER_Y = 32;

float curW = EYE_W, curHL = EYE_H, curHR = EYE_H;
float curOffX = 0, curOffY = 0;
float tgtW = EYE_W, tgtH = EYE_H;
float tgtOffX = 0, tgtOffY = 0;

unsigned long lastFaceFrame = 0;
unsigned long nextBlinkTime = 0, blinkStartTime = 0;
unsigned long nextLookTime = 0;
unsigned long nextSpecialTime = 0, specialStartTime = 0;
unsigned long lastActivityTime = 0;

bool isBlinking = false;
SpecialAction currentSpecial = SP_NONE;

const unsigned long FACE_FRAME_MS = 33;      // ~30 fps
const unsigned long BLINK_DURATION = 130;
const unsigned long DROWSY_AFTER = 120000;   // eyes droop after 2 min idle

unsigned long moodResetTime = 0;

void setFace(FaceMood mood) {
  currentMood = mood;
  if (mood != FACE_IDLE) {
    currentSpecial = SP_NONE;
    lastActivityTime = millis();
  }
}

void updateFaceTargets() {
  switch (currentMood) {
    case FACE_IDLE:
      tgtW = EYE_W; tgtH = EYE_H; tgtOffY = 0;
      break;
    case FACE_ALERT:
      tgtW = EYE_W + 8; tgtH = EYE_H + 8; tgtOffX = 0; tgtOffY = -2;
      break;
    case FACE_CAPTURE:
      tgtW = EYE_W + 6; tgtH = EYE_H + 6; tgtOffX = 0; tgtOffY = 0;
      break;
    case FACE_THINKING:
      tgtW = EYE_W - 2; tgtH = EYE_H - 2; tgtOffX = 4; tgtOffY = -7;
      break;
    case FACE_SENDING:
      tgtW = EYE_W; tgtH = EYE_H - 4; tgtOffY = 0;
      break;
    case FACE_HAPPY:
      tgtW = EYE_W + 2; tgtH = EYE_H; tgtOffX = 0; tgtOffY = 0;
      break;
    case FACE_SAD:
      tgtW = EYE_W; tgtH = EYE_H - 6; tgtOffX = 0; tgtOffY = 4;
      break;
    case FACE_SLEEPY:
      tgtW = EYE_W; tgtH = 7; tgtOffX = 0; tgtOffY = 3;
      break;
  }
}

void drawEye(int cx, int cy, int w, int h) {
  if (w < 2) w = 2;
  if (h < 2) h = 2;
  int r = min(w, h) / 2;
  display.fillRoundRect(cx - w / 2, cy - h / 2, w, h, r, SSD1306_WHITE);
}

void drawFace() {
  unsigned long now = millis();
  if (now - lastFaceFrame < FACE_FRAME_MS) return;
  lastFaceFrame = now;

  updateFaceTargets();

  float hL = tgtH, hR = tgtH;
  bool drowsy = false;

  if (currentMood == FACE_IDLE) {
    if (now - lastActivityTime > DROWSY_AFTER) {
      drowsy = true;
      tgtH = EYE_H * 0.55;
      hL = hR = tgtH;
    }

    // subtle "breathing"
    float breathe = sin(now / 1400.0) * 1.2;
    tgtW = EYE_W + breathe;

    if (currentSpecial == SP_NONE && now >= nextSpecialTime) {
      int pick = random(0, 5);
      currentSpecial = (SpecialAction)(SP_WINK + pick);
      specialStartTime = now;
      nextSpecialTime = now + random(9000, 18000);
    }

    if (currentSpecial != SP_NONE) {
      unsigned long el = now - specialStartTime;

      switch (currentSpecial) {
        case SP_WINK:                       // right eye only
          if (el < 400) {
            float r = (el < 200) ? (1.0 - el / 200.0) : ((el - 200) / 200.0);
            if (r < 0.08) r = 0.08;
            hR = tgtH * r;
          } else currentSpecial = SP_NONE;
          break;

        case SP_ROLL:                       // circular sweep
          if (el < 1400) {
            float a = (el / 1400.0) * TWO_PI;
            tgtOffX = cos(a) * 9;
            tgtOffY = sin(a) * 6;
          } else {
            tgtOffX = 0; tgtOffY = 0;
            currentSpecial = SP_NONE;
          }
          break;

        case SP_YAWN:
          if (el < 1600) {
            float p = el / 1600.0;
            float s = sin(p * PI);
            tgtW = EYE_W + s * 10;
            hL = hR = tgtH + s * 12;
            tgtOffY = -s * 3;
          } else {
            tgtOffY = 0;
            currentSpecial = SP_NONE;
          }
          break;

        case SP_SQUINT:
          if (el < 1200) {
            float p = el / 1200.0;
            float s = sin(p * PI);
            hL = hR = tgtH * (1.0 - s * 0.6);
            tgtOffX = s * 6;
          } else {
            tgtOffX = 0;
            currentSpecial = SP_NONE;
          }
          break;

        case SP_DOUBLE_BLINK:
          if (el < 500) {
            float ph = fmod(el, 250.0) / 250.0;
            float r = (ph < 0.5) ? (1.0 - ph * 2) : ((ph - 0.5) * 2);
            if (r < 0.08) r = 0.08;
            hL = hR = tgtH * r;
          } else currentSpecial = SP_NONE;
          break;

        default: currentSpecial = SP_NONE; break;
      }
    }

    if (currentSpecial == SP_NONE) {
      if (!isBlinking && now >= nextBlinkTime) {
        isBlinking = true;
        blinkStartTime = now;
      }
      if (isBlinking && now - blinkStartTime > BLINK_DURATION) {
        isBlinking = false;
        nextBlinkTime = now + (drowsy ? random(900, 2000) : random(2200, 5200));
      }
      if (isBlinking) {
        unsigned long el = now - blinkStartTime;
        float half = BLINK_DURATION / 2.0;
        float r = (el < half) ? (1.0 - el / half) : ((el - half) / half);
        if (r < 0.08) r = 0.08;
        hL = hR = tgtH * r;
      }

      // glance around, 8 directions
      if (now >= nextLookTime) {
        int d = random(0, 8);
        switch (d) {
          case 0: tgtOffX = -9; tgtOffY = 0;  break;
          case 1: tgtOffX =  9; tgtOffY = 0;  break;
          case 2: tgtOffX =  0; tgtOffY = -6; break;
          case 3: tgtOffX =  0; tgtOffY =  5; break;
          case 4: tgtOffX = -7; tgtOffY = -4; break;
          case 5: tgtOffX =  7; tgtOffY = -4; break;
          default: tgtOffX = 0; tgtOffY = 0;  break;
        }
        nextLookTime = now + random(1600, 4200);
      }
    }
  } else {
    isBlinking = false;
    currentSpecial = SP_NONE;
  }

  if (currentMood == FACE_SENDING) {
    tgtOffX = (now / 220 % 2 == 0) ? -6 : 6;
  }

  // ease toward targets
  curW    += (tgtW    - curW)    * 0.35;
  curHL   += (hL      - curHL)   * 0.45;
  curHR   += (hR      - curHR)   * 0.45;
  curOffX += (tgtOffX - curOffX) * 0.22;
  curOffY += (tgtOffY - curOffY) * 0.25;

  display.clearDisplay();

  int totalW = (int)curW * 2 + EYE_GAP;
  int leftX  = SCREEN_WIDTH / 2 - totalW / 2 + (int)curW / 2 + (int)curOffX;
  int rightX = leftX + (int)curW + EYE_GAP;
  int eyeY   = FACE_CENTER_Y + (int)curOffY;

  drawEye(leftX,  eyeY, (int)curW, (int)curHL);
  drawEye(rightX, eyeY, (int)curW, (int)curHR);

  // carve the eyes into crescents for happy / sad
  if (currentMood == FACE_HAPPY) {
    int cut = eyeY - (int)curHL / 2 + (int)(curHL * 0.45);
    display.fillRect(0, cut, SCREEN_WIDTH, SCREEN_HEIGHT - cut, SSD1306_BLACK);
  }
  if (currentMood == FACE_SAD) {
    int cut = eyeY - (int)curHL / 2 + (int)(curHL * 0.5);
    display.fillRect(0, 0, SCREEN_WIDTH, cut, SSD1306_BLACK);
  }

  display.display();
}

// Only used during boot, to show the IP address.
void showBootText(String line1, String line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2 != "") display.println(line2);
  display.display();
}

// ===========================================================================
// State
// ===========================================================================

long lastUpdateId = 0;
unsigned long lastPollTime = 0;
const unsigned long POLL_INTERVAL = 3000;

unsigned long lastPirTrigger = 0;
const unsigned long PIR_COOLDOWN = 30000;

SemaphoreHandle_t camMutex;
httpd_handle_t stream_httpd = NULL;

uint8_t* prevFrame = NULL;
const int MD_W = 80, MD_H = 60;
unsigned long lastMotionCheck = 0;
unsigned long lastMotionTrigger = 0;
bool motionBaselineReady = false;

// ===========================================================================
// Camera
// ===========================================================================

void discardFrames(int count) {
  for (int i = 0; i < count; i++) {
    camera_fb_t* tmp = esp_camera_fb_get();
    if (tmp) esp_camera_fb_return(tmp);
    delay(50);
  }
}

// Mean brightness 0-255, or -1 on failure.
int measureBrightness() {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return -1;

  s->set_framesize(s, FRAMESIZE_QVGA);
  delay(200);
  discardFrames(2);

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("[brightness] frame grab failed"); return -1; }

  // Size the buffer from the frame itself. Hardcoding dimensions here
  // corrupts the heap when the sensor hasn't switched yet.
  size_t rgbLen = (size_t)fb->width * fb->height * 3;
  uint8_t* rgb = (uint8_t*)ps_malloc(rgbLen);
  if (!rgb) {
    Serial.println("[brightness] PSRAM alloc failed");
    esp_camera_fb_return(fb);
    return -1;
  }

  int avg = -1;
  if (fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb)) {
    uint64_t sum = 0; uint32_t samples = 0;
    for (size_t i = 0; i + 2 < rgbLen; i += 3 * 16) {   // sample every 16th px
      sum += (rgb[i] + rgb[i+1] + rgb[i+2]) / 3;
      samples++;
    }
    if (samples > 0) avg = (int)(sum / samples);
  } else {
    Serial.println("[brightness] JPEG decode failed");
  }

  free(rgb);
  esp_camera_fb_return(fb);
  return avg;
}

// Grab a QQVGA frame and downsample to MD_W x MD_H grayscale.
bool grabMotionFrame(uint8_t* out) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return false;

  s->set_framesize(s, FRAMESIZE_QQVGA);
  delay(80);
  discardFrames(1);   // first frame after a size change is the OLD size

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return false;

  int fw = fb->width;
  int fh = fb->height;
  if (fw <= 0 || fh <= 0) { esp_camera_fb_return(fb); return false; }

  size_t rgbLen = (size_t)fw * fh * 3;
  uint8_t* rgb = (uint8_t*)ps_malloc(rgbLen);
  if (!rgb) { esp_camera_fb_return(fb); return false; }

  bool ok = false;
  if (fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, rgb)) {
    for (int y = 0; y < MD_H; y++) {
      int sy = (int)((long)y * fh / MD_H);
      if (sy >= fh) sy = fh - 1;
      for (int x = 0; x < MD_W; x++) {
        int sx = (int)((long)x * fw / MD_W);
        if (sx >= fw) sx = fw - 1;
        size_t idx = ((size_t)sy * fw + sx) * 3;
        out[y * MD_W + x] = (rgb[idx] + rgb[idx+1] + rgb[idx+2]) / 3;
      }
    }
    ok = true;
  }

  free(rgb);
  esp_camera_fb_return(fb);
  return ok;
}

// Mean absolute difference against the previous frame, or -1 on failure.
int detectMotion() {
  if (!prevFrame) return -1;

  uint8_t* cur = (uint8_t*)ps_malloc(MD_W * MD_H);
  if (!cur) return -1;

  if (!grabMotionFrame(cur)) { free(cur); return -1; }

  if (!motionBaselineReady) {
    memcpy(prevFrame, cur, MD_W * MD_H);
    motionBaselineReady = true;
    free(cur);
    return 0;
  }

  uint32_t diffSum = 0;
  for (int i = 0; i < MD_W * MD_H; i++) {
    int d = (int)cur[i] - (int)prevFrame[i];
    diffSum += (d < 0) ? -d : d;
  }
  int avgDiff = diffSum / (MD_W * MD_H);

  memcpy(prevFrame, cur, MD_W * MD_H);
  free(cur);
  return avgDiff;
}

camera_fb_t* captureHighRes() {
  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, CAPTURE_FRAMESIZE);
    delay(300);
    discardFrames(WARMUP_FRAMES);   // let AE/AWB settle
  }
  camera_fb_t* fb = NULL;
  for (int i = 0; i < 5; i++) {
    fb = esp_camera_fb_get();
    if (fb) break;
    Serial.printf("capture attempt %d failed, retrying\n", i + 1);
    delay(500);
  }
  return fb;
}

void restoreStreamMode() {
  sensor_t* s = esp_camera_sensor_get();
  if (s) { s->set_framesize(s, STREAM_FRAMESIZE); delay(100); }
}

// ===========================================================================
// Network
// ===========================================================================

// Base64 into PSRAM. Arduino String silently fails once the heap runs low,
// producing a request with an empty image field and no error.
char* encodeBase64ToPsram(const uint8_t* data, size_t len, size_t* outLen) {
  size_t cap = ((len + 2) / 3) * 4 + 8;
  char* b64 = (char*)ps_malloc(cap);
  if (!b64) return NULL;

  static const char* TBL =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  size_t o = 0;
  for (size_t i = 0; i < len; i += 3) {
    uint32_t v = data[i] << 16;
    if (i + 1 < len) v |= data[i+1] << 8;
    if (i + 2 < len) v |= data[i+2];
    b64[o++] = TBL[(v >> 18) & 0x3F];
    b64[o++] = TBL[(v >> 12) & 0x3F];
    b64[o++] = (i + 1 < len) ? TBL[(v >> 6) & 0x3F] : '=';
    b64[o++] = (i + 2 < len) ? TBL[v & 0x3F] : '=';
  }
  b64[o] = '\0';
  *outLen = o;
  return b64;
}

String askOllamaAboutImage(camera_fb_t* fb) {
  Serial.printf("[mem] heap: %d, psram: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

  size_t o = 0;
  char* b64 = encodeBase64ToPsram(fb->buf, fb->len, &o);
  if (!b64) { Serial.println("[error] base64 PSRAM alloc failed"); return "(out of memory)"; }
  Serial.printf("base64 encoded: %d chars\n", o);

  // Two things matter here:
  //   1. /api/chat, not /api/generate — some vision models ignore the image
  //      entirely on the generate endpoint.
  //   2. Keep the prompt ASCII. Raw UTF-8 in the JSON body breaks parsing on
  //      the ESP32 side; ask for the target language in the instruction instead.
  const char* head =
    "{\"model\":\"" VLM_MODEL "\",\"stream\":false,\"think\":false,\"messages\":[{"
    "\"role\":\"user\","
    "\"content\":\"You are a security scout robot. Look at this photo and report "
    "what you see in ONE short sentence. Focus on: are there people (how many, "
    "what they are doing), and what notable objects or furniture are visible. "
    "Be concrete and specific.\","
    "\"images\":[\"";
  const char* tail = "\"]}]}";

  size_t headLen = strlen(head), tailLen = strlen(tail);
  size_t bodyLen = headLen + o + tailLen;

  char* body = (char*)ps_malloc(bodyLen + 1);
  if (!body) {
    Serial.println("[error] request body PSRAM alloc failed");
    free(b64); return "(out of memory)";
  }
  memcpy(body, head, headLen);
  memcpy(body + headLen, b64, o);
  memcpy(body + headLen + o, tail, tailLen);
  body[bodyLen] = '\0';
  free(b64);

  Serial.printf("request body: %d bytes\n", bodyLen);

  HTTPClient http;
  http.begin(ollama_url);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Connection", "close");
  http.setTimeout(180000);
  http.setConnectTimeout(15000);

  int httpCode = http.POST((uint8_t*)body, bodyLen);
  Serial.printf("ollama HTTP %d\n", httpCode);
  free(body);

  String answer = "";
  if (httpCode > 0) {
    String response = http.getString();
    JsonDocument resDoc;
    DeserializationError error = deserializeJson(resDoc, response);
    if (!error) {
      answer = resDoc["message"]["content"].as<String>();   // /api/chat path
      if (answer == "null" || answer.length() == 0) answer = "(empty response)";
    } else {
      Serial.print("JSON parse error: "); Serial.println(error.c_str());
      answer = "(parse failed)";
    }
  } else {
    Serial.printf("ollama request failed: %d\n", httpCode);
    answer = "(no response)";
  }
  http.end();
  return answer;
}

bool sendMultipartPhoto(bool useHttps, const char* urlStr, String fieldNameExtra,
                        String extraValue, String photoFieldName, camera_fb_t* fb) {
  String boundary = "ESP32CamBoundary";

  String preamble = "";
  preamble += "--" + boundary + "\r\n";
  preamble += "Content-Disposition: form-data; name=\"" + fieldNameExtra + "\"\r\n\r\n";
  preamble += extraValue + "\r\n";
  preamble += "--" + boundary + "\r\n";
  preamble += "Content-Disposition: form-data; name=\"" + photoFieldName + "\"; filename=\"photo.jpg\"\r\n";
  preamble += "Content-Type: image/jpeg\r\n\r\n";
  String postamble = "\r\n--" + boundary + "--\r\n";

  size_t totalLen = preamble.length() + fb->len + postamble.length();
  uint8_t* body = (uint8_t*)ps_malloc(totalLen);
  if (!body) { Serial.println("multipart alloc failed"); return false; }

  size_t offset = 0;
  memcpy(body + offset, preamble.c_str(), preamble.length()); offset += preamble.length();
  memcpy(body + offset, fb->buf, fb->len);                    offset += fb->len;
  memcpy(body + offset, postamble.c_str(), postamble.length());

  // Plain WiFiClient on purpose: WiFiClientSecure reserves a large TLS buffer
  // in internal RAM, which is scarce right after a capture.
  HTTPClient http;
  WiFiClient plainClient;
  http.begin(plainClient, urlStr);
  delay(200);

  http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
  http.addHeader("Connection", "close");
  http.setTimeout(60000);
  http.setConnectTimeout(15000);
  Serial.printf("[archive] sending %d bytes\n", totalLen);

  int httpCode = http.POST(body, totalLen);
  Serial.printf("[archive] HTTP %d\n", httpCode);

  bool success = (httpCode == 200);
  if (!success) Serial.println("response: " + http.getString());

  free(body);
  http.end();
  return success;
}

bool sendPhotoToOne(camera_fb_t* fb, String caption, const char* chatId) {
  String url = "https://api.telegram.org/bot" + String(telegram_token) + "/sendPhoto";
  String boundary = "ESP32CamBoundary";

  String preamble = "";
  preamble += "--" + boundary + "\r\n";
  preamble += "Content-Disposition: form-data; name=\"chat_id\"\r\n\r\n";
  preamble += String(chatId) + "\r\n";
  preamble += "--" + boundary + "\r\n";
  preamble += "Content-Disposition: form-data; name=\"caption\"\r\n\r\n";
  preamble += caption + "\r\n";
  preamble += "--" + boundary + "\r\n";
  preamble += "Content-Disposition: form-data; name=\"photo\"; filename=\"photo.jpg\"\r\n";
  preamble += "Content-Type: image/jpeg\r\n\r\n";
  String postamble = "\r\n--" + boundary + "--\r\n";

  size_t totalLen = preamble.length() + fb->len + postamble.length();
  uint8_t* body = (uint8_t*)ps_malloc(totalLen);
  if (!body) { Serial.println("telegram alloc failed"); return false; }

  size_t offset = 0;
  memcpy(body + offset, preamble.c_str(), preamble.length()); offset += preamble.length();
  memcpy(body + offset, fb->buf, fb->len);                    offset += fb->len;
  memcpy(body + offset, postamble.c_str(), postamble.length());

  WiFiClientSecure client;
  client.setInsecure();

  HTTPClient http;
  http.begin(client, url);
  http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
  http.setTimeout(45000);

  int httpCode = http.POST(body, totalLen);
  Serial.printf("  -> HTTP %d\n", httpCode);

  bool success = (httpCode == 200);
  if (!success) Serial.println("  telegram: " + http.getString());

  free(body);
  http.end();
  return success;
}

bool sendPhotoToTelegram(camera_fb_t* fb, String caption) {
  bool allOk = true;
  for (int i = 0; i < PHOTO_RECIPIENT_COUNT; i++) {
    Serial.printf("telegram %d/%d\n", i + 1, PHOTO_RECIPIENT_COUNT);
    if (!sendPhotoToOne(fb, caption, PHOTO_RECIPIENTS[i])) allOk = false;
    delay(500);   // stay under the rate limit
  }
  return allOk;
}

bool sendToStorageServer(camera_fb_t* fb, String description) {
  return sendMultipartPhoto(false, storage_url, "description", description, "photo", fb);
}

// ===========================================================================
// Scout cycle
// ===========================================================================

void doScoutCycle(String triggerSource = "Telegram", bool sendTelegram = true) {
  setFace(FACE_CAPTURE);
  drawFace();

  xSemaphoreTake(camMutex, portMAX_DELAY);

  if (BRIGHTNESS_CHECK) {
    int brightness = measureBrightness();
    Serial.printf("[brightness] avg=%d (threshold=%d, trigger=%s)\n",
                  brightness, BRIGHTNESS_THRESHOLD, triggerSource.c_str());
    if (brightness >= 0 && brightness < BRIGHTNESS_THRESHOLD) {
      Serial.println("[brightness] too dark, skipping capture");
      restoreStreamMode();
      xSemaphoreGive(camMutex);
      setFace(FACE_SLEEPY);
      moodResetTime = millis() + 3000;
      motionBaselineReady = false;
      return;
    }
  }

  camera_fb_t* fb = captureHighRes();
  if (!fb) {
    Serial.println("capture failed");
    restoreStreamMode();
    xSemaphoreGive(camMutex);
    setFace(FACE_SAD);
    moodResetTime = millis() + 3000;
    motionBaselineReady = false;
    return;
  }
  Serial.printf("captured %d bytes (trigger: %s)\n", fb->len, triggerSource.c_str());

  // Copy to PSRAM and release the framebuffer immediately so the MJPEG
  // stream can resume while inference runs.
  size_t imgLen = fb->len;
  uint8_t* imgBuf = (uint8_t*)ps_malloc(imgLen);
  if (imgBuf) memcpy(imgBuf, fb->buf, imgLen);
  esp_camera_fb_return(fb);
  restoreStreamMode();
  xSemaphoreGive(camMutex);

  if (!imgBuf) {
    Serial.println("image copy PSRAM alloc failed");
    setFace(FACE_SAD);
    moodResetTime = millis() + 3000;
    motionBaselineReady = false;
    return;
  }

  camera_fb_t localFb;
  localFb.buf = imgBuf;
  localFb.len = imgLen;

  setFace(FACE_THINKING);
  drawFace();
  String answer = askOllamaAboutImage(&localFb);
  Serial.println("=== model output ===");
  Serial.println(answer);

  setFace(FACE_SENDING);
  drawFace();

  // Archive first (plain HTTP), Telegram second (HTTPS).
  bool storageSent = sendToStorageServer(&localFb, answer);
  Serial.println(storageSent ? "archived" : "archive failed");

  // Auto-captures skip Telegram, so a dead archive server would otherwise
  // go unnoticed. Warn at most once every 30 minutes.
  static unsigned long lastStorageAlert = 0;
  if (!storageSent && millis() - lastStorageAlert > 1800000) {
    lastStorageAlert = millis();
    WiFiClientSecure ac;
    ac.setInsecure();
    HTTPClient ah;
    String au = "https://api.telegram.org/bot" + String(telegram_token) +
                "/sendMessage?chat_id=" + String(telegram_chat_id) +
                "&text=" + "[ScoutBot] archive server unreachable";
    ah.begin(ac, au);
    ah.GET();
    ah.end();
    Serial.println("[alert] archive failure reported via telegram");
  }

  bool telegramSent = true;
  if (sendTelegram) {
    String caption = "[" + triggerSource + "] " + answer;
    telegramSent = sendPhotoToTelegram(&localFb, caption);
    Serial.println(telegramSent ? "telegram sent" : "telegram partially failed");
  } else {
    Serial.println("(auto capture: telegram skipped)");
  }

  free(imgBuf);

  setFace((telegramSent && storageSent) ? FACE_HAPPY : FACE_SAD);
  moodResetTime = millis() + 3500;

  // The scene just changed; don't treat that as motion.
  motionBaselineReady = false;
}

// ===========================================================================
// Telegram polling
// ===========================================================================

bool checkTelegramForCommand() {
  WiFiClientSecure client;
  client.setInsecure();

  String url = "https://api.telegram.org/bot" + String(telegram_token) +
               "/getUpdates?offset=" + String(lastUpdateId + 1) + "&timeout=0";

  HTTPClient http;
  http.begin(client, url);
  http.setTimeout(10000);

  int httpCode = http.GET();
  bool triggered = false;

  if (httpCode == 200) {
    String response = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, response)) {
      JsonArray results = doc["result"];
      for (size_t i = 0; i < results.size(); i++) {
        JsonObject update = results[i];

        // Advance the offset regardless of the allowlist, otherwise a
        // rejected message gets re-fetched forever.
        if (update["update_id"].is<long>()) {
          long id = update["update_id"];
          if (id > lastUpdateId) lastUpdateId = id;
        }

        if (update["message"].is<JsonObject>()) {
          JsonObject message = update["message"];

          long long fromId = 0;
          if (message["from"]["id"].is<long long>()) {
            fromId = message["from"]["id"].as<long long>();
          }

          if (message["text"].is<const char*>()) {
            String text = message["text"].as<String>();

            if (!isAllowedUser(fromId)) {
              Serial.printf("[blocked] %lld: %s\n", fromId, text.c_str());
              continue;
            }

            Serial.printf("message from %lld: %s\n", fromId, text.c_str());
            if (text == CMD_PHOTO_1 || text == CMD_PHOTO_2) triggered = true;
          }
        }
      }
    }
  } else {
    Serial.printf("getUpdates failed: %d\n", httpCode);
  }

  http.end();
  return triggered;
}

// ===========================================================================
// MJPEG stream server (port 81)
// ===========================================================================

#define STREAM_BOUNDARY "123456789000000000000987654321"
#define STREAM_CONTENT_TYPE "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY
static const char* STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t* fb = NULL;
  esp_err_t res = ESP_OK;
  char part_buf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;

  while (true) {
    xSemaphoreTake(camMutex, portMAX_DELAY);
    fb = esp_camera_fb_get();
    xSemaphoreGive(camMutex);

    if (!fb) { res = ESP_FAIL; break; }

    res = httpd_resp_send_chunk(req, "\r\n--" STREAM_BOUNDARY "\r\n", strlen("\r\n--" STREAM_BOUNDARY "\r\n"));
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, 64, STREAM_PART, fb->len);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len);

    xSemaphoreTake(camMutex, portMAX_DELAY);
    esp_camera_fb_return(fb);
    xSemaphoreGive(camMutex);

    if (res != ESP_OK) break;
    delay(60);
  }
  return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
  const char* html =
    "<html><body style='margin:0;background:#111;display:flex;justify-content:center;align-items:center;min-height:100vh;'>"
    "<img src='/stream' style='width:90vw;max-width:960px;height:auto;'>"
    "</body></html>";
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, html, strlen(html));
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 81;

  httpd_uri_t index_uri  = { .uri = "/",       .method = HTTP_GET, .handler = index_handler,  .user_ctx = NULL };
  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    httpd_register_uri_handler(stream_httpd, &stream_uri);
    Serial.println("stream server up on :81");
  } else {
    Serial.println("stream server failed to start");
  }
}

// ===========================================================================
// setup / loop
// ===========================================================================

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Gives you a window to hit upload if the sketch locks up the USB port.
  Serial.println("Ready to upload in 5 seconds...");
  for (int i = 5; i > 0; i--) { Serial.printf("%d...\n", i); delay(1000); }

  randomSeed(esp_random());
  camMutex = xSemaphoreCreateMutex();

  Wire.begin(I2C_SDA, I2C_SCL);
  pinMode(PIR_PIN, INPUT);

  // The AXP2101 gates the camera's power rails. esp_camera_init() fails
  // outright unless ALDO1/2/4 are enabled first.
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    Serial.println("PMU init failed");
  } else {
    PMU.setDC3Voltage(3100);   PMU.enableDC3();
    PMU.setALDO1Voltage(1500); PMU.enableALDO1();   // camera core
    PMU.setALDO2Voltage(3000); PMU.enableALDO2();   // camera
    PMU.setALDO4Voltage(3000); PMU.enableALDO4();   // camera
    PMU.setALDO3Voltage(3300); PMU.enableALDO3();
    PMU.setBLDO1Voltage(3300); PMU.enableBLDO1();
    Serial.println("PMU ok, camera rails on");
  }
  delay(1000);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init failed");
  } else {
    showBootText("Booting...");
  }

  Serial.printf("[pre-camera] heap: %d, psram: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());
  if (psramFound()) {
    Serial.printf("[psram] %d total, %d free\n", ESP.getPsramSize(), ESP.getFreePsram());
  } else {
    Serial.println("[psram] NOT DETECTED - check Tools > PSRAM = OPI PSRAM");
  }

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;   config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM; config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM; config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;   config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.jpeg_quality = 8;            // lower = better quality
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  // Fall back through smaller sizes if allocation fails. The deinit() is
  // mandatory — retrying without it fails every time.
  framesize_t tryList[]  = { FRAMESIZE_SXGA, FRAMESIZE_XGA, FRAMESIZE_SVGA, FRAMESIZE_VGA };
  const char* tryNames[] = { "SXGA", "XGA", "SVGA", "VGA" };
  esp_err_t err = ESP_FAIL;

  for (int i = 0; i < 4; i++) {
    config.frame_size = tryList[i];
    Serial.printf("[camera] trying %s (psram: %d)\n", tryNames[i], ESP.getFreePsram());
    err = esp_camera_init(&config);
    if (err == ESP_OK) { Serial.printf("[camera] %s ok\n", tryNames[i]); break; }
    Serial.printf("[camera] %s failed (0x%x), deinit and retry\n", tryNames[i], err);
    esp_camera_deinit();
    delay(300);
  }

  if (err != ESP_OK) {
    Serial.println("camera init failed at every resolution");
    showBootText("Camera FAILED");
    return;
  }
  Serial.printf("[post-camera] heap: %d, psram: %d\n", ESP.getFreeHeap(), ESP.getFreePsram());

  sensor_t* sensor = esp_camera_sensor_get();
  if (sensor != NULL) {
    sensor->set_whitebal(sensor, 1);
    sensor->set_awb_gain(sensor, 1);
    sensor->set_wb_mode(sensor, 0);
    sensor->set_brightness(sensor, 0);
    sensor->set_contrast(sensor, 1);
    sensor->set_saturation(sensor, 0);
    sensor->set_sharpness(sensor, 1);
    sensor->set_gain_ctrl(sensor, 1);
    sensor->set_exposure_ctrl(sensor, 1);
    sensor->set_gainceiling(sensor, GAINCEILING_4X);
    sensor->set_lenc(sensor, 1);      // lens shading correction
    sensor->set_bpc(sensor, 1);       // black pixel correction
    sensor->set_wpc(sensor, 1);       // white pixel correction
    sensor->set_raw_gma(sensor, 1);
    sensor->set_dcw(sensor, 1);
    sensor->set_vflip(sensor, 1);
    sensor->set_quality(sensor, 8);
    Serial.println("sensor tuned");
    sensor->set_framesize(sensor, STREAM_FRAMESIZE);
    delay(200);
    discardFrames(2);
  }

  if (MOTION_DETECT_ENABLED) {
    prevFrame = (uint8_t*)ps_malloc(MD_W * MD_H);
    if (prevFrame) {
      Serial.printf("[motion] buffer %dx%d, threshold=%d, cooldown=%ds\n",
                    MD_W, MD_H, MOTION_THRESHOLD, MOTION_COOLDOWN / 1000);
    } else {
      Serial.println("[motion] buffer alloc failed - disabled");
    }
  }

  // Wi-Fi comes last: once its stack claims internal RAM, allocating a large
  // camera DMA buffer fails.
  WiFi.begin(ssid, password);
  Serial.println("connecting to wifi...");
  showBootText("Connecting WiFi...");
  int tryCount = 0;
  while (WiFi.status() != WL_CONNECTED && tryCount < 30) {
    delay(500); Serial.print("."); tryCount++;
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nwifi failed");
    showBootText("WiFi FAILED");
    return;
  }
  Serial.println("\nwifi ok, IP: " + WiFi.localIP().toString());
  showBootText("WiFi OK!", WiFi.localIP().toString());
  delay(2500);

  startCameraServer();

  Serial.println("=== ready (telegram + motion) ===");
  Serial.printf("=== brightness threshold: %d ===\n", BRIGHTNESS_THRESHOLD);
  Serial.printf("=== %d allowed users, %d photo recipients ===\n",
                ALLOWED_USER_COUNT, PHOTO_RECIPIENT_COUNT);

  unsigned long now = millis();
  setFace(FACE_IDLE);
  lastActivityTime = now;
  nextBlinkTime   = now + 1500;
  nextLookTime    = now + 2500;
  nextSpecialTime = now + 8000;
  lastMotionTrigger = now - MOTION_COOLDOWN;   // allow a trigger right away
}

void loop() {
  unsigned long now = millis();

  drawFace();

  if (moodResetTime != 0 && now >= moodResetTime) {
    moodResetTime = 0;
    setFace(FACE_IDLE);
    lastActivityTime = now;
    nextBlinkTime   = now + 1200;
    nextLookTime    = now + 2000;
    nextSpecialTime = now + 6000;
  }

  if (now - lastPollTime >= POLL_INTERVAL) {
    lastPollTime = now;
    if (checkTelegramForCommand()) {
      Serial.println(">>> telegram command received");
      doScoutCycle("Telegram", true);
    }
  }

#if MOTION_DETECT_ENABLED
  if (prevFrame && moodResetTime == 0 && currentMood == FACE_IDLE &&
      now - lastMotionCheck >= MOTION_CHECK_INTERVAL) {
    lastMotionCheck = now;

    // Hold the mutex only for the frame grab. restoreStreamMode() would add
    // a 100 ms delay inside the critical section and stall the MJPEG stream.
    xSemaphoreTake(camMutex, portMAX_DELAY);
    int diff = detectMotion();
    sensor_t* s = esp_camera_sensor_get();
    if (s) s->set_framesize(s, STREAM_FRAMESIZE);
    xSemaphoreGive(camMutex);

    if (diff < 0) {
      Serial.println("[motion] frame grab failed");
    } else if (diff >= MOTION_THRESHOLD) {
      if (now - lastMotionTrigger >= MOTION_COOLDOWN) {
        Serial.printf("[motion] delta=%d, capturing\n", diff);
        lastMotionTrigger = now;
        setFace(FACE_ALERT);
        drawFace();
        doScoutCycle("Motion", MOTION_SEND_TELEGRAM);
      }
    }
  }
#endif

#if PIR_ENABLED
  if (digitalRead(PIR_PIN) == HIGH && (now - lastPirTrigger >= PIR_COOLDOWN)) {
    lastPirTrigger = now;
    doScoutCycle("PIR-Motion", true);
  }
#endif
}
