#pragma once
#include <Arduino.h>
class ESP32PWM { public: static void allocateTimer(int) {} };
class Servo {
  int pin=-1;
public:
  void setPeriodHertz(int) {}
  void setTimerWidth(int) {}
  void attach(int p, int, int) { pin=p; ledcAttachChannel(p,50,14,0); }
  bool attached() const { return pin>=0; }
  void detach() { ledcDetach(pin); pin=-1; }
  void writeMicroseconds(int us) { ledcWrite(pin,us); }
};
