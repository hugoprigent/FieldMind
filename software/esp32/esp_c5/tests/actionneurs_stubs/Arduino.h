#pragma once
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <string>
constexpr uint8_t D0=1, D1=0, D2=25, D3=7, D4=23, D7=12, D9=9;
constexpr int INPUT=0, OUTPUT=1, HIGH=1, LOW=0, pdPASS=1, RISING=1;
#define IRAM_ATTR
extern uint32_t horloge;
extern uint32_t decalageMicros; // micros() = horloge * 1000 + decalageMicros
extern int modes[32];
extern void (*interruptions[32])();
extern bool creationTache;
extern int niveaux[32];
inline uint32_t millis() { return horloge; }
inline void delay(uint32_t ms) { horloge += ms; }
inline void pinMode(uint8_t pin, int mode) { modes[pin]=mode; }
inline uint32_t micros() { return horloge * 1000u + decalageMicros; }
inline int digitalPinToInterrupt(uint8_t pin) { return pin; }
inline void attachInterrupt(int pin, void (*isr)(), int) { interruptions[pin]=isr; }
inline void detachInterrupt(int pin) { interruptions[pin]=nullptr; }
inline void digitalWrite(uint8_t pin, int niveau) { niveaux[pin]=niveau; }
inline int xTaskCreate(void (*)(void *), const char *, int, void *, int, void *) {
  return creationTache ? pdPASS : 0;
}
struct Console {
  bool connectee=true;
  std::string entree, sortie;
  void begin(int) {}
  explicit operator bool() const { return connectee; }
  int available() const { return static_cast<int>(entree.size()); }
  int read() { const int c=entree.front(); entree.erase(0,1); return c; }
  void println(const char *s) { sortie += s; sortie += '\n'; }
  size_t write(const uint8_t *b, size_t n) { sortie.append(reinterpret_cast<const char *>(b), n); return n; }
  void printf(const char *s, ...) {
    char b[1024]; va_list args; va_start(args,s);
    vsnprintf(b,sizeof(b),s,args); va_end(args); sortie += b;
  }
};
extern Console Serial;
bool ledcAttachChannel(uint8_t, uint32_t, uint8_t, uint8_t);
bool ledcWrite(uint8_t, uint32_t);
bool ledcDetach(uint8_t);
uint32_t ledcRead(uint8_t);
