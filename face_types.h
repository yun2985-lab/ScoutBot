#ifndef FACE_TYPES_H
#define FACE_TYPES_H

// These enums live here rather than in the .ino because the Arduino IDE
// generates function prototypes at the top of the sketch, ahead of any
// type declarations in the file itself.

enum FaceMood {
  FACE_IDLE, FACE_CAPTURE, FACE_THINKING, FACE_SENDING,
  FACE_HAPPY, FACE_SAD, FACE_SLEEPY, FACE_ALERT
};

enum SpecialAction {
  SP_NONE, SP_WINK, SP_ROLL, SP_YAWN, SP_SQUINT, SP_DOUBLE_BLINK
};

#endif
