// FieldMind : logiciel ESP32 du rover, XIAO ESP32-C5 / carte A7.
// Programme principal (env esp_c5) : capteurs, ecran, deux moteurs et servo.
// USB 115200 : m 10, s -10, v 30, centre, m 0, sens 0/1, frein 0/1, stop, etat.
// ROVER_3 (pilotage par le Pi) : p g d servo, hb, wd ms, inv ms, tel hz, imu hz, lacet0.
// Gauche : PWM D0/GPIO1, DIR D1/GPIO0 inverse ; droite : PWM D9/GPIO9, DIR D4/GPIO23.
// BRAKE commun D10/GPIO10 ; servo D7/GPIO12, centre au demarrage.
// Controleur commun avec le test : actionneurs.cpp, tache separee de l'I2C.
// Traction a zero au demarrage ; les vitesses et angles physiques ne sont pas mesures.
// XIAO ESP32C5, bus I2C sur SDA = D5 / GPIO24 et SCL = D6 / GPIO11 :
// écran OLED 128 x 64, GY-521 (MPU-6050), BMP280 et Modulino Knob (facultatif).
//
// Sans Knob, l'écran affiche en continu les mesures des capteurs.
// Accueil : les yeux du robot suivent l'inclinaison de la carte, et louchent si on la secoue.
// Le Knob navigue : tourner choisit, appui court ouvre ou revient au menu,
// appui long ramène à l'accueil. Dans une appli, tourner passe à l'appli voisine.
// Menu : Capteurs (valeurs de l'IMU), Niveau (bulle), Diagnostic (bus I2C), Infos.
// En mode complet/capteurs, le moniteur reçoit les mesures toutes les 500 ms.
// Commande série « d » : diagnostic du bus (balayages répétés, lecture des chip-ID, niveau des lignes).
// Un composant absent est recherché à nouveau toutes les 2 s : on peut l'enficher à chaud.
// ROVER_3 (env esp_c5) : le GY-521 est lu par une tâche « centrale » à au moins 25 Hz, ou à la
// cadence demandée par « imu <hz> » (lignes I avec lacet intégré), hors du rythme de l'écran.
// Les lectures I2C sont protégées par un verrou ; les lignes USB passent par console_serie.h.

#include <Arduino.h>
#include <Wire.h>
#include <esp_mac.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>
#include "moteur_zs_x11d1.h"
#include "moteur_sorties.h"
#include "actionneurs.h"
#include "console_serie.h"
#ifdef FIELDMIND_ROVER_COMPLET
#include "centrale_lacet.h"
#include "pilotage_rover3.h"
#include "drone_wifi.h"
#endif

constexpr uint8_t PIN_SDA = moteur::PIN_I2C_SDA; // D5 / GPIO24
constexpr uint8_t PIN_SCL = moteur::PIN_I2C_SCL; // D6 / GPIO11 (console USB)
constexpr uint32_t I2C_HZ = 400000;        // repasser à 100000 si le bus fait des erreurs
constexpr uint8_t OLED_ROTATION = 2;       // écran monté tête en bas sur la carte ; 0 s'il est à l'endroit
constexpr int8_t KNOB_SENS = 1;            // -1 si le menu défile à l'envers
constexpr uint8_t KNOB_PAS = 1;            // crans du Knob pour un pas de menu
constexpr bool AXES_ECHANGES = false;      // true si les yeux et la bulle bougent sur le mauvais axe
constexpr int8_t SENS_X = 1, SENS_Y = 1;   // -1 pour inverser un axe de l'écran
constexpr uint32_t APPUI_LONG_MS = 700;
constexpr uint32_t VEILLE_MENU_MS = 30000; // retour à l'accueil sans action dans le menu
constexpr uint32_t KNOB_MS = 10;
constexpr uint32_t IMAGE_MS = 40;          // ~25 images/s
constexpr uint32_t JOURNAL_MS = 500;
constexpr uint32_t REESSAI_MS = 2000;

// L'écran règle lui-même l'horloge du bus pendant et après chaque envoi : on lui impose la nôtre.
Adafruit_SSD1306 oled(128, 64, &Wire, -1, I2C_HZ, I2C_HZ);

enum class Etat { Absent, Douteux, Ok };
const char *nomEtat(Etat e) { return e == Etat::Ok ? "OK" : e == Etat::Douteux ? "DOUTEUX" : "ABSENT"; }

struct {
  uint8_t adresse = 0;
  bool pret = false;
} ecran;

struct {
  uint8_t adresse = 0, whoAmI = 0, echecs = 0;
  bool pret = false, lu = false;
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, temp = 0;
} imu;

struct {
  uint8_t chipId = 0, echecs = 0;
  bool pret = false, lu = false, vu76 = false, vu77 = false;
  uint16_t T1 = 0, P1 = 0;
  int16_t T2 = 0, T3 = 0, P2 = 0, P3 = 0, P4 = 0, P5 = 0, P6 = 0, P7 = 0, P8 = 0, P9 = 0;
  float pression = 0, temp = 0;
} baro;

struct {
  uint8_t adresse = 0, echecs = 0;
  bool pret = false, appuye = false;
  int16_t position = 0;
} knob;

struct {
  uint8_t adresses[12];
  uint8_t n = 0;
  uint32_t date = 0;
} bus;

enum class Vue { Accueil, Menu, Capteurs, Niveau, Diagnostic, Infos };
Vue vue = Vue::Accueil;
int choix = 0;
bool busBloque = false;                    // un composant a été perdu : le pilote I2C peut rester figé
uint32_t derniereAction = 0;

// ------------------------------------------------------------------ I2C
// Wire verrouille chaque transaction, mais pas la lecture du tampon de réception qui suit
// requestFrom() : la tâche centrale et loop() prennent ce verrou autour de chaque lecture.
SemaphoreHandle_t verrouI2C = nullptr;
struct VerrouI2C {
  VerrouI2C() { if (verrouI2C) xSemaphoreTakeRecursive(verrouI2C, portMAX_DELAY); }
  ~VerrouI2C() { if (verrouI2C) xSemaphoreGiveRecursive(verrouI2C); }
};

bool repond(uint8_t adr) {
  Wire.beginTransmission(adr);
  return Wire.endTransmission() == 0;
}

bool lireRegistres(uint8_t adr, uint8_t reg, uint8_t *dst, size_t n) {
  VerrouI2C verrou;
  Wire.beginTransmission(adr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(adr, (uint8_t)n) != n) return false;
  for (size_t i = 0; i < n; i++) dst[i] = Wire.read();
  return true;
}

int lireRegistre(uint8_t adr, uint8_t reg) {
  uint8_t v;
  return lireRegistres(adr, reg, &v, 1) ? v : -1;
}

bool ecrireRegistre(uint8_t adr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(adr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

const char *nomAdresse(uint8_t a) {
  switch (a) {
    case 0x3C: case 0x3D: return "ecran OLED (SSD1306/SSD1315)";
    case 0x68: case 0x69: return "GY-521 (MPU-6050)";
    case 0x3A: case 0x3B: return "Modulino Knob";
    case 0x76: case 0x77: return "BMP280";
    default: return "inconnu";
  }
}

const char *nomCourt(uint8_t a) {
  switch (a) {
    case 0x3C: case 0x3D: return "OLED";
    case 0x68: case 0x69: return "GY-521";
    case 0x3A: case 0x3B: return "Knob";
    case 0x76: case 0x77: return "BMP280";
    default: return "?";
  }
}

void balayerBus(bool journal) {
  if (journal) console::println("\n--- Balayage I2C (SDA D5/GPIO24, SCL D6/GPIO11) ---");
  bus.n = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (!repond(a)) continue;
    if (bus.n < sizeof bus.adresses) bus.adresses[bus.n++] = a;
    if (journal) console::printf("  0x%02X : %s\n", a, nomAdresse(a));
  }
  bus.date = millis();
  if (journal && bus.n == 0) {
    console::println("  Aucun appareil : verifier 3V3, GND, SDA/SCL et les resistances de tirage.");
  }
}

// ------------------------------------------------------------------ OLED
void initEcran() {
  for (uint8_t a : {0x3C, 0x3D}) {
    if (repond(a) && oled.begin(SSD1306_SWITCHCAPVCC, a, false, false)) {
      ecran.adresse = a;
      ecran.pret = true;
      oled.setRotation(OLED_ROTATION);
      oled.cp437(true);                    // pour le signe degré (0xF8)
      oled.setTextColor(SSD1306_WHITE);
      console::printf("[OLED]   trouve en 0x%02X, initialise\n", a);
      return;
    }
  }
  console::println("[OLED]   absent (0x3C / 0x3D)");
}

// ------------------------------------------------------------------ GY-521
// Lecture directe des registres : fonctionne avec le MPU-6050 et ses clones (MPU-6500...),
// que la bibliothèque Adafruit refuse.
const char *nomImu(uint8_t who) {
  switch (who) {
    case 0x68: return "MPU-6050";
    case 0x70: return "MPU-6500";
    case 0x71: return "MPU-9250";
    case 0x73: return "MPU-9255";
    case 0x98: return "clone (0x98)";
    default: return "puce inconnue";
  }
}

void initImu() {
  for (uint8_t a : {0x68, 0x69}) {
    if (!repond(a)) continue;
    int who = lireRegistre(a, 0x75);
    imu.adresse = a;
    imu.whoAmI = who < 0 ? 0 : who;
    ecrireRegistre(a, 0x6B, 0x01);   // sortie de veille, horloge sur le gyro X
    delay(50);
    ecrireRegistre(a, 0x1A, 0x03);   // filtre passe-bas ~44 Hz
    ecrireRegistre(a, 0x1B, 0x00);   // gyro +-250 deg/s
    ecrireRegistre(a, 0x1C, 0x00);   // accéléro +-2 g
    imu.pret = true;
    imu.echecs = 0;
    console::printf("[GY-521] trouve en 0x%02X, WHO_AM_I = 0x%02X (%s)\n", a, imu.whoAmI, nomImu(imu.whoAmI));
    return;
  }
  console::println("[GY-521] absent (0x68 / 0x69)");
}

void lireImu() {
  uint8_t b[14];
  imu.lu = lireRegistres(imu.adresse, 0x3B, b, sizeof b);
  if (!imu.lu) {
    if (++imu.echecs >= 5) {
      imu.pret = false;
      busBloque = true;
      console::println("[GY-521] perdu");
    }
    return;
  }
  imu.echecs = 0;
  auto s16 = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  imu.ax = s16(0) / 16384.0f;
  imu.ay = s16(2) / 16384.0f;
  imu.az = s16(4) / 16384.0f;
  imu.temp = imu.whoAmI == 0x68 ? s16(6) / 340.0f + 36.53f : s16(6) / 333.87f + 21.0f;
  imu.gx = s16(8) / 131.0f;
  imu.gy = s16(10) / 131.0f;
  imu.gz = s16(12) / 131.0f;
}

Etat etatImu() {
  if (!imu.pret) return Etat::Absent;
  float g = sqrtf(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
  bool plausible = imu.lu && g > 0.8f && g < 1.2f && imu.temp > -20 && imu.temp < 70;
  return plausible ? Etat::Ok : Etat::Douteux;    // au repos, la norme de l'accélération vaut ~1 g
}

// Accélération dans le repère de l'écran (x vers la droite, y vers le bas), en g.
// Retourner l'écran de 180° inverse aussi ses deux axes.
void inclinaison(float &x, float &y) {
  const int8_t demiTour = OLED_ROTATION == 2 ? -1 : 1;
  x = (AXES_ECHANGES ? imu.ay : imu.ax) * SENS_X * demiTour;
  y = (AXES_ECHANGES ? imu.ax : imu.ay) * SENS_Y * demiTour;
}

// ------------------------------------------------------------------ centrale rapide (ROVER_3)
// Tâche séparée de loop() : l'écran (environ 25 ms par image sur le bus) ne retarde plus
// les lectures. Cadence de lecture : max(25 Hz, imu) ; lignes I seulement si imu > 0.
bool centraleEnTache = false;              // sinon, IMU lue dans loop() comme avant
#ifdef FIELDMIND_ROVER_COMPLET
constexpr uint32_t CENTRALE_MIN_HZ = 25;   // rythme d'origine (IMAGE_MS)
constexpr uint32_t REPOS_TRACTION_MS = 1000;
centrale::Lacet lacet;

void tacheCentrale(void *) {
  TickType_t reveil = xTaskGetTickCount();
  uint32_t precedentUs = 0;
  bool precedent = false;
  rover3::Cadence lignes;
  for (;;) {
    const uint32_t hz = actionneurs::cadenceCentrale();
    const uint32_t lectureHz = hz > CENTRALE_MIN_HZ ? hz : CENTRALE_MIN_HZ;
    if (xTaskDelayUntil(&reveil, pdMS_TO_TICKS(rover3::periodeMs(lectureHz))) == pdFALSE)
      reveil = xTaskGetTickCount();        // en retard (bus lent) : pas de rafale de rattrapage
    if (lignes.hz != hz) lignes.regler(hz, millis());
    if (actionneurs::remiseLacetDemandee()) lacet.remettreAZero();
    if (!imu.pret) { precedent = false; continue; }
    lireImu();
    if (!imu.lu) continue;
    const uint32_t us = micros(), ms = millis();
    const float dt = precedent ? (us - precedentUs) * 1e-6f : 0.0f;
    precedentUs = us;
    precedent = true;
    const float a[3] = {imu.ax, imu.ay, imu.az}, g[3] = {imu.gx, imu.gy, imu.gz};
    lacet.ajouter(g[0], g[1], g[2], dt, actionneurs::tractionAuRepos(ms, REPOS_TRACTION_MS));
    if (hz && (hz >= CENTRALE_MIN_HZ || lignes.due(ms))) {
      char texte[112];
      const int n = rover3::formaterCentrale(texte, sizeof texte, ms, a, g, lacet.lacet);
      if (n > 0 && static_cast<size_t>(n) < sizeof texte) console::ecrire(texte, static_cast<size_t>(n));
    }
  }
}
#endif

// ------------------------------------------------------------------ BMP280
const char *nomBaro(uint8_t id) {
  switch (id) {
    case 0x58: return "BMP280";
    case 0x56: case 0x57: return "BMP280 (echantillon)";
    case 0x60: return "BME280";
    default: return "puce inconnue";
  }
}

// Pilote maison : si la broche SDO du module flotte, la puce passe toute seule de 0x76 à 0x77.
// Chaque échange essaie donc les deux adresses, et une lecture n'est retenue que si deux
// lectures de suite donnent les mêmes octets.
bool baroLire(uint8_t reg, uint8_t *dst, size_t n) {
  uint8_t a[24], b[24];
  for (int essai = 0; essai < 10; essai++) {
    for (uint8_t adr : {0x76, 0x77}) {
      if (!lireRegistres(adr, reg, a, n)) continue;
      (adr == 0x76 ? baro.vu76 : baro.vu77) = true;
      for (uint8_t adr2 : {0x76, 0x77}) {
        if (!lireRegistres(adr2, reg, b, n)) continue;
        if (!memcmp(a, b, n)) {
          memcpy(dst, a, n);
          return true;
        }
        break;
      }
    }
  }
  return false;
}

bool baroEcrire(uint8_t reg, uint8_t val) {
  for (int essai = 0; essai < 10; essai++) {
    for (uint8_t adr : {0x76, 0x77}) {
      uint8_t relu;
      if (ecrireRegistre(adr, reg, val) && baroLire(reg, &relu, 1) && relu == val) return true;
    }
  }
  return false;
}

void initBaro() {
  if (!repond(0x76) && !repond(0x77) && !repond(0x76) && !repond(0x77)) {
    console::println("[BMP280] absent (0x76 / 0x77)");
    return;
  }
  uint8_t id, c[24];
  if (!baroLire(0xD0, &id, 1) || !baroLire(0x88, c, sizeof c)) {
    console::println("[BMP280] repond mais ses lectures sont instables");
    return;
  }
  baro.chipId = id;
  auto u16 = [&](int i) { return (uint16_t)(c[i] | (c[i + 1] << 8)); };
  baro.T1 = u16(0);  baro.T2 = u16(2);  baro.T3 = u16(4);
  baro.P1 = u16(6);  baro.P2 = u16(8);  baro.P3 = u16(10); baro.P4 = u16(12); baro.P5 = u16(14);
  baro.P6 = u16(16); baro.P7 = u16(18); baro.P8 = u16(20); baro.P9 = u16(22);
  if (baro.T1 == 0 || baro.T1 == 0xFFFF || baro.P1 == 0 || baro.P1 == 0xFFFF) {
    console::println("[BMP280] coefficients de calibration illisibles");
    return;
  }
  // Mode normal, température x2, pression x16, filtre x4, une mesure toutes les 62,5 ms
  // (réglage « station météo » de la fiche technique).
  if (!baroEcrire(0xF5, 0x28) || !baroEcrire(0xF4, 0x57)) {
    console::println("[BMP280] configuration refusee");
    return;
  }
  baro.pret = true;
  baro.echecs = 0;
  console::printf("[BMP280] trouve, chip-ID = 0x%02X (%s), adresse %s\n", id, nomBaro(id),
                baro.vu76 && baro.vu77 ? "instable 0x76/0x77 : broche SDO en l'air" : baro.vu76 ? "0x76" : "0x77");
}

// Compensation entière de la fiche technique Bosch (§3.11.3).
void compenserBaro(int32_t adcT, int32_t adcP, float &temp, float &pres) {
  int32_t v1 = (((adcT >> 3) - ((int32_t)baro.T1 << 1)) * baro.T2) >> 11;
  int32_t dt = (adcT >> 4) - (int32_t)baro.T1;
  int32_t v2 = (((dt * dt) >> 12) * baro.T3) >> 14;
  int32_t tFine = v1 + v2;
  temp = ((tFine * 5 + 128) >> 8) / 100.0f;
  int64_t p1 = (int64_t)tFine - 128000, p2 = p1 * p1 * baro.P6;
  p2 += (p1 * baro.P5) << 17;
  p2 += (int64_t)baro.P4 << 35;
  p1 = ((p1 * p1 * baro.P3) >> 8) + ((p1 * baro.P2) << 12);
  p1 = (((int64_t)1 << 47) + p1) * baro.P1 >> 33;
  if (p1 == 0) {
    pres = NAN;
    return;
  }
  int64_t p = 1048576 - adcP;
  p = (((p << 31) - p2) * 3125) / p1;
  int64_t q1 = ((int64_t)baro.P9 * (p >> 13) * (p >> 13)) >> 25, q2 = ((int64_t)baro.P8 * p) >> 19;
  p = ((p + q1 + q2) >> 8) + ((int64_t)baro.P7 << 4);
  pres = p / 25600.0f;
}

void lireBaro() {
  uint8_t d[6];
  baro.lu = baroLire(0xF7, d, 6);
  if (baro.lu) {
    int32_t adcP = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
    int32_t adcT = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
    baro.lu = adcT != 0x80000 && adcP != 0x80000;   // 0x80000 : mesure pas encore faite
    if (baro.lu) compenserBaro(adcT, adcP, baro.temp, baro.pression);
  }
  if (baro.lu) {
    baro.echecs = 0;
  } else if (++baro.echecs >= 25) {               // ~1 s sans lecture valable
    baro.pret = false;
    console::println("[BMP280] perdu");
  }
}

Etat etatBaro() {
  if (!baro.pret) return Etat::Absent;
  bool plausible = baro.lu && baro.pression > 800 && baro.pression < 1100 && baro.temp > -20 && baro.temp < 70;
  return plausible ? Etat::Ok : Etat::Douteux;
}

// ------------------------------------------------------------------ diagnostic du bus (commande « d »)
// Balaye le bus 200 fois et compte les réponses par adresse : une adresse qui répond par
// intermittence, ou une adresse inattendue, trahit une broche d'adresse ou de mode mal reliée.
void diagnostiquerBus() {
  console::println("\n=== Diagnostic du bus I2C ===");
  console::printf("Lignes au repos : SDA = %d, SCL = %d (1 attendu)\n", digitalRead(PIN_SDA), digitalRead(PIN_SCL));
  for (uint32_t hz : {100000u, 400000u}) {
    Wire.setClock(hz);
    console::printf("-- %u kHz --\n", (unsigned)(hz / 1000));
    static uint16_t vus[128];
    memset(vus, 0, sizeof vus);
    const int tours = 200;
    int lesDeux = 0;
    for (int i = 0; i < tours; i++) {
      for (uint8_t a = 0x08; a < 0x78; a++)
        if (repond(a)) vus[a]++;
      if (repond(0x76) && repond(0x77)) lesDeux++;
    }
    for (uint8_t a = 0x08; a < 0x78; a++)
      if (vus[a]) console::printf("  0x%02X : %3u / %d  %s\n", a, vus[a], tours, nomAdresse(a));
    console::printf("  0x76 et 0x77 a la suite : %d / %d\n", lesDeux, tours);
    for (uint8_t a : {0x76, 0x77}) {
      if (!vus[a]) continue;
      int ok = 0, id58 = 0, calibIdentique = 0;
      uint8_t ref[24], c[24];
      bool refOk = lireRegistres(a, 0x88, ref, sizeof ref);
      for (int i = 0; i < 50; i++) {
        int id = lireRegistre(a, 0xD0);
        if (id >= 0) ok++;
        if (id == 0x58) id58++;
        if (refOk && lireRegistres(a, 0x88, c, sizeof c) && !memcmp(ref, c, sizeof c)) calibIdentique++;
      }
      console::printf("  0x%02X : chip-ID lu %d / 50 (dont %d = 0x58), calibration relue identique %d / 50\n",
                    a, ok, id58, calibIdentique);
      if (refOk) {
        char texte[24 * 3 + 24] = "     calibration :";
        size_t n = strlen(texte);
        for (uint8_t b : ref) n += snprintf(texte + n, sizeof texte - n, " %02X", b);
        console::println(texte);
      }
    }
  }
  Wire.setClock(I2C_HZ);
  console::println("=== fin ===\n");
}

// Commande « b » : registres bruts du BMP280, lus par le pilote tolérant, et leur conversion.
void diagnostiquerBaro() {
  console::println("\n=== BMP280 brut ===");
  if (!baro.pret) {
    console::println("  non initialise");
    return;
  }
  console::printf("  adresses vues : 0x76 %s, 0x77 %s\n", baro.vu76 ? "oui" : "non", baro.vu77 ? "oui" : "non");
  console::printf("  T1..T3 = %u %d %d | P1..P9 = %u %d %d %d %d %d %d %d %d\n", baro.T1, baro.T2, baro.T3,
                baro.P1, baro.P2, baro.P3, baro.P4, baro.P5, baro.P6, baro.P7, baro.P8, baro.P9);
  for (int k = 0; k < 6; k++) {
    uint8_t r[3] = {}, d[6] = {};
    bool ok = baroLire(0xF3, r, 3) && baroLire(0xF7, d, 6);
    int32_t adcP = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4);
    int32_t adcT = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4);
    float temp, pres;
    compenserBaro(adcT, adcP, temp, pres);
    console::printf("  %s status %02X ctrl %02X config %02X | adcP %6ld adcT %6ld -> %.2f hPa %.2f C\n",
                  ok ? "lu   " : "ECHEC", r[0], r[1], r[2], (long)adcP, (long)adcT, pres, temp);
    delay(100);
  }
  console::println("=== fin ===\n");
}

// ------------------------------------------------------------------ Modulino Knob
// Trame du Knob : adresse de configuration, position (int16 petit-boutiste), bouton (0 ou 1).
bool lireKnob() {
  VerrouI2C verrou;
  if (Wire.requestFrom(knob.adresse, (uint8_t)4) != 4) {
    while (Wire.available()) Wire.read();
    return false;
  }
  Wire.read();
  uint8_t bas = Wire.read(), haut = Wire.read();
  knob.position = (int16_t)(bas | (haut << 8));
  knob.appuye = Wire.read() != 0;
  return true;
}

void initKnob() {
  for (uint8_t a : {0x3A, 0x3B}) {
    if (!repond(a)) continue;
    knob.adresse = a;
    if (lireKnob()) {
      knob.pret = true;
      knob.echecs = 0;
      console::printf("[KNOB]   trouve en 0x%02X, position %d\n", a, knob.position);
      return;
    }
  }
  console::println("[KNOB]   absent (0x3A / 0x3B)");
}

struct Commandes {
  int pas = 0;
  bool court = false, longue = false;
  bool actif() const { return pas || court || longue; }
};

Commandes lireCommandes() {
  static bool suivi = false, etaitAppuye = false, longueSignalee = false;
  static int16_t reference = 0;
  static uint32_t debutAppui = 0;
  Commandes c;
  if (!knob.pret) {
    suivi = false;
    return c;
  }
  if (!lireKnob()) {
    if (++knob.echecs >= 5) {
      knob.pret = false;
      busBloque = true;
      vue = Vue::Accueil;                  // plus rien pour naviguer
      console::println("[KNOB]   perdu");
    }
    return c;
  }
  knob.echecs = 0;
  if (!suivi) {                            // première lecture : rien à signaler
    suivi = true;
    reference = knob.position;
    etaitAppuye = knob.appuye;
    longueSignalee = true;
    return c;
  }

  int pas = (int16_t)(knob.position - reference) / KNOB_PAS;
  if (pas) {
    reference += pas * KNOB_PAS;
    c.pas = pas * KNOB_SENS;
  }

  uint32_t t = millis();
  if (knob.appuye && !etaitAppuye) {
    debutAppui = t;
    longueSignalee = false;
  }
  if (knob.appuye && !longueSignalee && t - debutAppui >= APPUI_LONG_MS) {
    c.longue = true;
    longueSignalee = true;
  }
  if (!knob.appuye && etaitAppuye && !longueSignalee) c.court = true;
  etaitAppuye = knob.appuye;
  return c;
}

// ------------------------------------------------------------------ dessin : outils
void texteCentre(const char *s, int y) {
  int16_t x1, y1;
  uint16_t w, h;
  oled.getTextBounds(s, 0, y, &x1, &y1, &w, &h);
  oled.setCursor((128 - w) / 2, y);
  oled.print(s);
}

void pastille(int x, char lettre, bool ok) {
  if (ok) {
    oled.fillRoundRect(x, 0, 9, 9, 2, SSD1306_WHITE);
    oled.setTextColor(SSD1306_BLACK);
  } else {
    oled.drawRoundRect(x, 0, 9, 9, 2, SSD1306_WHITE);
  }
  oled.setCursor(x + 2, 1);
  oled.print(lettre);
  oled.setTextColor(SSD1306_WHITE);
}

// Titre, temps depuis le démarrage, pastilles IMU, baromètre et Knob (pleines = présents).
void barreEtat(const char *titre) {
  oled.setCursor(0, 1);
  oled.print(titre);
  unsigned s = millis() / 1000;
  oled.setCursor(62, 1);
  if (s < 3600) oled.printf("%02u:%02u", s / 60, s % 60);
  else oled.printf("%uh%02u", s / 3600, s / 60 % 60);
  pastille(94, 'I', imu.pret);
  pastille(105, 'B', baro.pret);
  pastille(116, 'K', knob.pret);
  oled.drawFastHLine(0, 10, 128, SSD1306_WHITE);
}

// ------------------------------------------------------------------ icônes du menu (≈ 28 x 24)
void iconeCapteurs(int cx, int cy) {
  oled.drawRoundRect(cx - 14, cy - 11, 29, 23, 3, SSD1306_WHITE);
  int px = cx - 10, py = cy;
  for (int x = -10; x <= 10; x++) {
    int y = cy - (int)roundf(6 * sinf(x * 0.45f));
    oled.drawLine(px, py, cx + x, y, SSD1306_WHITE);
    px = cx + x;
    py = y;
  }
}

void iconeNiveau(int cx, int cy) {
  oled.drawCircle(cx, cy, 12, SSD1306_WHITE);
  oled.drawCircle(cx, cy, 4, SSD1306_WHITE);
  oled.drawFastHLine(cx - 12, cy, 5, SSD1306_WHITE);
  oled.drawFastHLine(cx + 8, cy, 5, SSD1306_WHITE);
  oled.drawFastVLine(cx, cy - 12, 5, SSD1306_WHITE);
  oled.drawFastVLine(cx, cy + 8, 5, SSD1306_WHITE);
  oled.fillCircle(cx + 5, cy - 5, 3, SSD1306_WHITE);
}

void iconeDiagnostic(int cx, int cy) {
  oled.drawRect(cx - 8, cy - 8, 17, 17, SSD1306_WHITE);
  oled.fillRect(cx - 4, cy - 4, 9, 9, SSD1306_WHITE);
  for (int d : {-5, 0, 5}) {
    oled.drawFastHLine(cx - 13, cy + d, 5, SSD1306_WHITE);
    oled.drawFastHLine(cx + 9, cy + d, 5, SSD1306_WHITE);
    oled.drawFastVLine(cx + d, cy - 13, 5, SSD1306_WHITE);
    oled.drawFastVLine(cx + d, cy + 9, 5, SSD1306_WHITE);
  }
}

void iconeInfos(int cx, int cy) {
  oled.drawCircle(cx, cy, 12, SSD1306_WHITE);
  oled.fillCircle(cx, cy - 6, 2, SSD1306_WHITE);
  oled.fillRect(cx - 1, cy - 2, 3, 9, SSD1306_WHITE);
}

struct Appli {
  const char *nom;
  Vue vue;
  void (*icone)(int, int);
};

const Appli APPLIS[] = {
  {"Capteurs", Vue::Capteurs, iconeCapteurs},
  {"Niveau", Vue::Niveau, iconeNiveau},
  {"Diagnostic", Vue::Diagnostic, iconeDiagnostic},
  {"Infos", Vue::Infos, iconeInfos},
};
constexpr int NB_APPLIS = sizeof APPLIS / sizeof APPLIS[0];

const char *nomVue(Vue v) {
  if (v == Vue::Accueil) return "Accueil";
  if (v == Vue::Menu) return "Menu";
  for (const Appli &a : APPLIS) if (a.vue == v) return a.nom;
  return "?";
}

// ------------------------------------------------------------------ écrans
void oeil(int cx, int cy, int h, bool etourdi, bool gauche) {
  if (etourdi) {                           // ">" pour l'œil gauche, "<" pour le droit
    int s = gauche ? 1 : -1;
    for (int e = -1; e <= 1; e++) {
      oled.drawLine(cx - 9 * s, cy - 9 + e, cx + 9 * s, cy + e, SSD1306_WHITE);
      oled.drawLine(cx + 9 * s, cy + e, cx - 9 * s, cy + 9 + e, SSD1306_WHITE);
    }
    return;
  }
  oled.fillRoundRect(cx - 14, cy - h / 2, 28, h, min(7, h / 2), SSD1306_WHITE);
}

void dessinerAccueil() {
  static float rx = 0, ry = 0;
  static uint32_t prochainClignement = 3000, finEtourdi = 0;
  uint32_t t = millis();
  barreEtat("FieldMind");

  // Le regard glisse vers le côté qui penche.
  float vx = 0, vy = 0;
  if (imu.pret) {
    float x, y;
    inclinaison(x, y);
    vx = constrain(-x * 20, -10.0f, 10.0f);
    vy = constrain(-y * 10, -5.0f, 5.0f);
    if (max(fabsf(imu.gx), max(fabsf(imu.gy), fabsf(imu.gz))) > 200) finEtourdi = t + 1200;
  }
  rx += (vx - rx) * 0.3f;
  ry += (vy - ry) * 0.3f;

  int h = 28;
  if (t >= prochainClignement) {
    if (t - prochainClignement < 140) h = 4;
    else prochainClignement = t + random(2500, 6000);
  }
  bool etourdi = t < finEtourdi;
  int cy = 36 + (int)ry;
  oeil(38 + (int)rx, cy, h, etourdi, true);
  oeil(90 + (int)rx, cy, h, etourdi, false);

  if (!imu.pret) texteCentre("GY-521 absent", 57);
}

void dessinerMenu() {
  barreEtat("Menu");
  const Appli &a = APPLIS[choix];
  a.icone(64, 28);
  oled.fillTriangle(5, 28, 11, 22, 11, 34, SSD1306_WHITE);
  oled.fillTriangle(122, 28, 116, 22, 116, 34, SSD1306_WHITE);
  texteCentre(a.nom, 45);
  for (int i = 0; i < NB_APPLIS; i++) {
    int x = 64 - (NB_APPLIS - 1) * 5 + i * 10;
    if (i == choix) oled.fillCircle(x, 59, 2, SSD1306_WHITE);
    else oled.drawPixel(x, 59, SSD1306_WHITE);
  }
}

bool imuAbsente() {
  if (imu.pret) return false;
  texteCentre("GY-521 absent", 32);
  return true;
}

void dessinerCapteurs() {
  barreEtat("Capteurs");
  oled.setCursor(0, 13);
  oled.printf("IMU %s", nomEtat(etatImu()));
  if (imu.pret) {
    float g = sqrtf(imu.ax * imu.ax + imu.ay * imu.ay + imu.az * imu.az);
    oled.setCursor(72, 13);
    oled.printf("%4.2f g", g);
    oled.setCursor(0, 22);
    oled.printf("a %+5.2f %+5.2f %+5.2f", imu.ax, imu.ay, imu.az);
    oled.setCursor(0, 31);
    oled.printf("w %+5.0f %+5.0f %+5.0f", imu.gx, imu.gy, imu.gz);
  }
  oled.drawFastHLine(0, 41, 128, SSD1306_WHITE);
  oled.setCursor(0, 44);
  oled.printf("BARO %s", nomEtat(etatBaro()));
  if (baro.pret) {
    oled.setCursor(0, 54);
    oled.printf("%7.1f hPa  %4.1f\xF8""C", baro.pression, baro.temp);
  }
}

void dessinerNiveau() {
  barreEtat("Niveau");
  if (imuAbsente()) return;
  const int cx = 36, cy = 38, r = 24;
  oled.drawCircle(cx, cy, r, SSD1306_WHITE);
  oled.drawCircle(cx, cy, 6, SSD1306_WHITE);
  oled.drawFastHLine(cx - r, cy, 5, SSD1306_WHITE);
  oled.drawFastHLine(cx + r - 4, cy, 5, SSD1306_WHITE);
  oled.drawFastVLine(cx, cy - r, 5, SSD1306_WHITE);
  oled.drawFastVLine(cx, cy + r - 4, 5, SSD1306_WHITE);

  // La bulle monte vers le côté haut : 0,5 g d'inclinaison (30°) la met au bord.
  float x, y;
  inclinaison(x, y);
  float bx = x * 2 * (r - 5), by = y * 2 * (r - 5);
  float d = sqrtf(bx * bx + by * by);
  if (d > r - 5) {
    bx *= (r - 5) / d;
    by *= (r - 5) / d;
  }
  oled.fillCircle(cx + (int)bx, cy + (int)by, 4, SSD1306_WHITE);

  float roulis = atan2f(imu.ay, imu.az) * RAD_TO_DEG;
  float tangage = atan2f(-imu.ax, sqrtf(imu.ay * imu.ay + imu.az * imu.az)) * RAD_TO_DEG;
  oled.setCursor(70, 15);
  oled.print(F("Roulis"));
  oled.setCursor(70, 25);
  oled.printf("%+6.1f\xF8", roulis);
  oled.setCursor(70, 37);
  oled.print(F("Tangage"));
  oled.setCursor(70, 47);
  oled.printf("%+6.1f\xF8", tangage);
  if (fabsf(roulis) < 1.5f && fabsf(tangage) < 1.5f) {
    oled.setCursor(70, 57);
    oled.print(F("A plat"));
  }
}

const char *verdictAdresse(uint8_t a) {
  switch (a) {
    case 0x3C: case 0x3D: return ecran.pret ? "OK" : "";
    case 0x68: case 0x69: return nomEtat(etatImu());
    case 0x3A: case 0x3B: return knob.pret ? "OK" : "";
    case 0x76: case 0x77: return nomEtat(etatBaro());
    default: return "";
  }
}

void dessinerDiagnostic() {
  barreEtat("Diagnostic");
  if (bus.n == 0) {
    texteCentre("Bus I2C vide", 32);
    return;
  }
  for (int i = 0; i < min<int>(bus.n, 4); i++) {
    uint8_t a = bus.adresses[i];
    oled.setCursor(0, 13 + 10 * i);
    oled.printf("0x%02X %-7s %s", a, nomCourt(a), verdictAdresse(a));
  }
  oled.setCursor(0, 55);
  if (bus.n > 4) oled.printf("+%u autres, %u kHz", bus.n - 4, (unsigned)(I2C_HZ / 1000));
  else oled.printf("%u sur le bus, %u kHz", bus.n, (unsigned)(I2C_HZ / 1000));
}

void dessinerInfos() {
  barreEtat("Infos");
  uint8_t m[6] = {};
  esp_read_mac(m, ESP_MAC_WIFI_STA);
  oled.setCursor(0, 14);
  oled.print(F("FieldMind  carte A7"));
  oled.setCursor(0, 24);
  oled.print(F("XIAO ESP32-C5"));
  oled.setCursor(0, 34);
  oled.printf("%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
  oled.setCursor(0, 44);
  oled.printf("RAM libre %u ko", (unsigned)(ESP.getFreeHeap() / 1024));
  oled.setCursor(0, 55);
  oled.print(F("Compile " __DATE__));
}

void dessiner() {
  oled.clearDisplay();
  switch (knob.pret ? vue : Vue::Capteurs) {   // sans Knob, pas de navigation : les mesures
    case Vue::Accueil: dessinerAccueil(); break;
    case Vue::Menu: dessinerMenu(); break;
    case Vue::Capteurs: dessinerCapteurs(); break;
    case Vue::Niveau: dessinerNiveau(); break;
    case Vue::Diagnostic: dessinerDiagnostic(); break;
    case Vue::Infos: dessinerInfos(); break;
  }
  oled.display();
}

// ------------------------------------------------------------------ navigation
void ouvrir(int i) {
  choix = (i % NB_APPLIS + NB_APPLIS) % NB_APPLIS;
  vue = APPLIS[choix].vue;
  if (vue == Vue::Diagnostic) balayerBus(false);
}

void naviguer(const Commandes &c) {
  uint32_t t = millis();
  if (c.actif()) derniereAction = t;
  if (c.longue) {
    vue = Vue::Accueil;
    return;
  }
  switch (vue) {
    case Vue::Accueil:
      if (c.pas || c.court) vue = Vue::Menu;
      break;
    case Vue::Menu:
      if (c.pas) choix = ((choix + c.pas) % NB_APPLIS + NB_APPLIS) % NB_APPLIS;
      if (c.court) ouvrir(choix);
      else if (t - derniereAction > VEILLE_MENU_MS) vue = Vue::Accueil;
      break;
    default:
      if (c.court) vue = Vue::Menu;
      else if (c.pas) ouvrir(choix + c.pas);
      break;
  }
}

// ------------------------------------------------------------------ programme
void setup() {
  verrouI2C = xSemaphoreCreateRecursiveMutex();
#ifdef FIELDMIND_ROVER_COMPLET
  actionneurs::demarrer(); // Fixe les sorties avant les operations capteurs.
  if (!actionneurs::demarrerTache())
    console::println("[DEFAUT] Tache actionneurs indisponible ; sorties coupees.");
#else
#ifdef FIELDMIND_MOTOR_CONTROL
  moteur::demarrer();
#endif
  Serial.begin(115200);
  for (uint32_t t0 = millis(); !Serial && millis() - t0 < 3000;) delay(10);
#ifdef FIELDMIND_DUAL_MOTOR
  moteur::demarrerConsoleAutonome();
#endif
#endif
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
  console::println("\n=== FieldMind : bureau d'accueil, carte capteurs A7 ===");
  balayerBus(true);
  initEcran();
  initImu();
  initBaro();
  initKnob();
#if defined(FIELDMIND_MOTOR_CONTROL) && !defined(FIELDMIND_DUAL_MOTOR)
  moteur::annoncer();
#endif
#ifdef FIELDMIND_ROVER_COMPLET
  centraleEnTache = xTaskCreate(tacheCentrale, "centrale", 4096, nullptr, 2, nullptr) == pdPASS;
  if (!centraleEnTache) console::println("[CENTRALE] Tache indisponible : IMU lue au rythme de l'ecran, sans lignes I.");
  drone::demarrerTache();
#endif
}

void loop() {
  static uint32_t dernierKnob = 0, derniereImage = 0, dernierJournal = 0, dernierEssai = 0;
  static bool led = false;
  uint32_t t = millis();

  // nouvelle recherche des composants absents (enfichage à chaud)
  if (t - dernierEssai > REESSAI_MS && !(ecran.pret && imu.pret && baro.pret && knob.pret)) {
    dernierEssai = t;
    if (busBloque) {                       // après une perte, le pilote ne repart qu'en relançant le bus
      busBloque = false;
      {
        VerrouI2C verrou;                  // la tâche centrale n'utilise pas le bus pendant ce temps
        Wire.end();
        Wire.begin(PIN_SDA, PIN_SCL, I2C_HZ);
      }
      console::println("[I2C]    bus relance");
    }
    if (!ecran.pret) initEcran();
    if (!imu.pret) initImu();
    if (!baro.pret) initBaro();
    if (!knob.pret) initKnob();
  }

#if defined(FIELDMIND_ROVER_COMPLET)
  // USB gere exclusivement par la tache actionneurs, jamais par les capteurs.
#elif defined(FIELDMIND_MOTOR_CONTROL)
  const char diagnostic = moteur::lireConsole();
  if (diagnostic == 'd') diagnostiquerBus();
  if (diagnostic == 'b') diagnostiquerBaro();
#else
  while (Serial.available()) {
    int ch = tolower(Serial.read());
    if (ch == 'd') diagnostiquerBus();
    if (ch == 'b') diagnostiquerBaro();
  }
#endif

  bool redessiner = false;
  if (t - dernierKnob >= KNOB_MS) {
    dernierKnob = t;
    Commandes c = lireCommandes();
    naviguer(c);
    redessiner = c.actif();
  }

  if (redessiner || t - derniereImage >= IMAGE_MS) {
    derniereImage = t;
    if (imu.pret && !centraleEnTache) lireImu();
    if (baro.pret) lireBaro();
    if (vue == Vue::Diagnostic && t - bus.date > 1000) balayerBus(false);
    if (ecran.pret) dessiner();
  }

  if (t - dernierJournal >= JOURNAL_MS) {
    dernierJournal = t;
    digitalWrite(LED_BUILTIN, (led = !led) ? HIGH : LOW);   // battement : le programme tourne
#ifdef FIELDMIND_MOTOR_CONTROL
    if (moteur::journalCapteursActif()) {
#endif
#ifdef FIELDMIND_ROVER_COMPLET
    if (actionneurs::cadenceCentrale() == 0)   // imu > 0 : lignes I de la tâche centrale à la place
#endif
    console::printf("IMU %-7s a=[%+5.2f %+5.2f %+5.2f] g  w=[%+7.1f %+7.1f %+7.1f] deg/s  %4.1f C"
                  " | BARO %-7s %7.2f hPa %5.1f C | KNOB %-6s pos %+5d %s | %s\n",
                  nomEtat(etatImu()), imu.ax, imu.ay, imu.az, imu.gx, imu.gy, imu.gz, imu.temp,
                  nomEtat(etatBaro()), baro.pression, baro.temp,
                  knob.pret ? "OK" : "ABSENT", knob.position, knob.appuye ? "appuye" : "      ", nomVue(vue));
#ifdef FIELDMIND_MOTOR_CONTROL
    }
#endif
  }
}

