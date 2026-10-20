#pragma once
// Sortie USB partagee par les taches actionneurs, capteurs et centrale.
// Chaque appel est mis en file d'un bloc : deux lignes ne s'entremelent jamais.
// Une seule tache d'ecriture appelle Serial.write ; les autres ne bloquent jamais
// sur l'USB (si l'hote ne lit plus, la file deborde et les lignes sont perdues,
// comptees par perdues()). Avant demarrer(), ecriture directe (une seule tache).
#include <Arduino.h>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#if defined(ARDUINO_ARCH_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#endif

namespace console {
constexpr size_t LIGNE_MAX = 320;

struct Etat {
#if defined(ARDUINO_ARCH_ESP32)
  RingbufHandle_t file = nullptr;
  SemaphoreHandle_t verrou = nullptr; // Repli sans tache d'ecriture.
#endif
  volatile uint32_t perdues = 0;
};
inline Etat &etat() { static Etat e; return e; }

inline void ecrire(const char *texte, size_t n) {
  if (!texte || !n) return;
#if defined(ARDUINO_ARCH_ESP32)
  if (RingbufHandle_t file = etat().file) {
    if (xRingbufferSend(file, texte, n, 0) != pdTRUE) etat().perdues = etat().perdues + 1;
    return;
  }
  if (SemaphoreHandle_t verrou = etat().verrou) {
    xSemaphoreTake(verrou, portMAX_DELAY);
    Serial.write(reinterpret_cast<const uint8_t *>(texte), n);
    xSemaphoreGive(verrou);
    return;
  }
#endif
  Serial.write(reinterpret_cast<const uint8_t *>(texte), n);
}

inline void print(const char *texte) { if (texte) ecrire(texte, std::strlen(texte)); }

inline void println(const char *texte = "") {
  char tampon[LIGNE_MAX];
  const int n = std::snprintf(tampon, sizeof tampon, "%s\n", texte ? texte : "");
  if (n > 0) ecrire(tampon, static_cast<size_t>(n) < sizeof tampon ? static_cast<size_t>(n) : sizeof tampon - 1);
}

inline void printf(const char *format, ...) __attribute__((format(printf, 1, 2)));
inline void printf(const char *format, ...) {
  char tampon[LIGNE_MAX];
  va_list args;
  va_start(args, format);
  const int n = std::vsnprintf(tampon, sizeof tampon, format, args);
  va_end(args);
  if (n <= 0) return;
  const size_t longueur = static_cast<size_t>(n) < sizeof tampon ? static_cast<size_t>(n) : sizeof tampon - 1;
  if (static_cast<size_t>(n) >= sizeof tampon) tampon[longueur - 1] = '\n'; // ligne tronquee
  ecrire(tampon, longueur);
}

inline uint32_t perdues() { return etat().perdues; }

inline size_t espaceLibre() {
#if defined(ARDUINO_ARCH_ESP32)
  if (RingbufHandle_t file = etat().file) return xRingbufferGetCurFreeSize(file);
#endif
  return 8192;
}

#if defined(ARDUINO_ARCH_ESP32)
inline void tacheEcriture(void *) {
  for (;;) {
    size_t n = 0;
    void *bloc = xRingbufferReceive(etat().file, &n, portMAX_DELAY);
    if (!bloc) continue;
    Serial.write(static_cast<const uint8_t *>(bloc), n); // seule tache qui ecrit sur l'USB
    vRingbufferReturnItem(etat().file, bloc);
  }
}
#endif

// Programmes avec plusieurs taches : file de 16 ko et tache d'ecriture.
// Repli si la file ou la tache manque : ecriture directe protegee par un mutex.
inline bool demarrer() {
#if defined(ARDUINO_ARCH_ESP32)
  if (etat().file) return true;
  if (!etat().verrou) etat().verrou = xSemaphoreCreateMutex();
  RingbufHandle_t file = xRingbufferCreate(16384, RINGBUF_TYPE_NOSPLIT);
  if (!file) return false;
  etat().file = file;
  if (xTaskCreate(tacheEcriture, "console", 3072, nullptr, 2, nullptr) != pdPASS) {
    etat().file = nullptr;
    vRingbufferDelete(file);
    return false;
  }
#endif
  return true;
}
}  // namespace console
