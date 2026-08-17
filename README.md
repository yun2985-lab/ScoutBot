# ScoutBot
<img width="3000" height="4000" alt="KakaoTalk_20260817_204838185" src="https://github.com/user-attachments/assets/a97c5343-be69-4f6c-a867-140a1e8c88c9" />
A DIY AI-powered scout robot built on the LilyGo T-Camera S3 (ESP32-S3), running
a **fully local** vision pipeline — no cloud APIs involved.

The robot watches a room, detects motion on-device, captures a photo, has a local
vision-language model describe what it sees, and reports back over Telegram while
archiving everything for later fine-tuning.

---

## What it does
<img width="2528" height="943" alt="image" src="https://github.com/user-attachments/assets/a2bb3886-874c-41a1-9655-161c19ccd258" />
<img width="2189" height="428" alt="image" src="https://github.com/user-attachments/assets/dadd6775-0875-479f-b02d-7d0148383528" />

```
[Trigger]                 [ESP32-S3]                      [Local PC]

Telegram command ─┐
  (whitelisted)    │
                   ├→ brightness check → SXGA capture → VLM ─┬→ Flask archive server
Motion detection  ─┘                                          │        ↓
  (frame diffing)                                             │   dataset folder
                                                              │        ↓
                                                              │   review web app
                                                              │        ↓
                                                              │   JSONL → LoRA
                                                              │
                                                              └→ Telegram (manual only)
```

- **On-device motion detection** — QQVGA frames downsampled to 80×60 grayscale,
  compared frame-to-frame. No PIR sensor needed.
- **Low-light gating** — measures average brightness before capturing; silently
  skips shots too dark to be useful as training data.
- **Local VLM inference** — images are sent to Ollama running on a PC on the same
  LAN. Nothing leaves the local network.
- **Animated OLED face** — the 128×64 display shows expressive eyes instead of
  text: blinking, glancing around, winking, yawning, and mood changes that track
  what the robot is doing.
- **Multi-user Telegram control** with an explicit allowlist.

---

## Hardware

| Component | Part |
|---|---|
| Board | LilyGo T-Camera S3 V1.6 (ESP32-S3, 8MB OPI PSRAM) |
| Camera | OV2640 (2MP) |
| Display | SSD1306 OLED 128×64 |
| Power management | AXP2101 PMU |

**Planned additions**

| Component | Purpose |
|---|---|
| INMP441 | I2S microphone — wake word + voice commands |
| MAX98357A + 8Ω speaker | I2S audio output — spoken responses |
| HC-SR04 / VL53L0X | Obstacle avoidance for the walking version |
| PCA9685 + 8× SG90 | Quadruped legs (2 DOF per leg) |

---

## Software stack

**Firmware** — Arduino IDE, ESP32 core 2.0.17, ArduinoJson, XPowersLib,
`esp_http_server`

**Inference** — Ollama with a local vision model, served over LAN

**Backend** — Python 3 + Flask
- archive server: receives photo + description, stores timestamped pairs
- review web app: browse captures, correct AI descriptions by hand
- dataset prep: converts corrected pairs to JSONL for fine-tuning

**Fine-tuning** — Unsloth + QLoRA on a single consumer GPU

---

## Notable implementation details

A few things that took real debugging to get right, in case they help someone
else building on this board:

**Power rails must be enabled explicitly.** The AXP2101 PMU gates the camera's
supply. `esp_camera_init()` fails outright unless ALDO1/2/4 are turned on via
XPowersLib first.

**Initialize the camera before Wi-Fi.** Once the Wi-Fi stack claims internal RAM,
allocating a large DMA framebuffer fails with `cam_dma_config failed`.

**Call `esp_camera_deinit()` between resolution fallback attempts.** Retrying
without it always fails, regardless of the requested frame size.

**Never hardcode buffer sizes for `fmt2rgb888()`.** The first frame after
`set_framesize()` still carries the *previous* resolution, so a hardcoded buffer
overflows and corrupts the heap. Always allocate from `fb->width` / `fb->height`,
and discard a frame or two after switching.

**Build large HTTP request bodies directly in PSRAM.** Using Arduino `String` or
ArduinoJson for base64 image payloads fails silently once the heap runs short —
the request goes out with the image field empty and no error is raised.

**Keep the camera mutex hold time short.** Motion detection, MJPEG streaming, and
capture all contend for the sensor. Holding the mutex through a full inference
round trip stalls the stream and delays command handling by tens of seconds.

**Move enums into a separate header.** The Arduino IDE injects auto-generated
function prototypes above everything else in a `.ino` file, so an enum used as a
parameter type appears undeclared even though the code is correct.

---

## Status

Working:

- [x] Motion-triggered and command-triggered capture
- [x] Local VLM description with structured prompting
- [x] Telegram reporting with user allowlist
- [x] PC-side archival and human review UI
- [x] Animated OLED face
- [x] Low-light rejection

In progress:

- [ ] Voice interaction — wake word, STT, TTS
- [ ] Dataset accumulation for LoRA fine-tuning
- [ ] Quadruped chassis and gait

---

## Why "JayJay"

The robot is named JayJay — that's the identity it already uses as a Telegram
bot, and what it gets called around the house. Adding voice interaction means it
should answer to that name directly.

---

## License

MIT
