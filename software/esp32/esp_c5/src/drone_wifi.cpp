#include "drone_wifi.h"
#include "console_serie.h"

#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiClient.h>
#include <atomic>
#include <cstring>
#include <strings.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace drone {
namespace {

constexpr size_t TAILLE_MAX_JPEG = 20480;
constexpr size_t TAILLE_RX_UDP = 1472;
constexpr uint32_t PERIODE_ETAT_MS = 500;
constexpr uint32_t PERIODE_SCAN_MS = 8000;
constexpr uint32_t PERIODE_SONDE_VIDEO_MS = 2000;
constexpr uint32_t PERIODE_CMD_VOL_MS = 50; // 20 Hz

SemaphoreHandle_t verrouConfig = nullptr;
char ssidCible[40] = "";       // vide = detection automatique d'un drone ouvert
char motDePasseCible[40] = ""; // vide = reseau ouvert
std::atomic<bool> demandeScan{true};
std::atomic<bool> demandeReconnexion{false};
std::atomic<bool> demandeDeconnexion{false};
std::atomic<bool> demandeEtat{false};
std::atomic<bool> fluxActif{true};

// Commandes de vol HD-DRONE (manette -100..+100 et drapeaux).
std::atomic<int> cmdGaz{0};
std::atomic<int> cmdLacet{0};
std::atomic<int> cmdTangage{0};
std::atomic<int> cmdRoulis{0};
std::atomic<uint8_t> drapeauVol{0}; // bit0=decoller, bit1=atterrir, bit2=stop, bit3=calibrer, bit4=sans_tete
std::atomic<uint32_t> finDrapeauMs{0};
std::atomic<bool> envoiVolActif{false};
const char *nomActionVol = "aucune";

// Etat courant et statistiques.
const char *etatCourant = "scan";
char ssidConnecte[40] = "";
char bssidConnecte[20] = "";
char ipLocale[20] = "";
char ipPasserelle[20] = "";
char protocoleDetecte[24] = "attente";
int canalWifi = 0;
int rssiDbm = 0;
uint16_t portVideo = 0;
uint16_t portCmd = 8888;
uint32_t paquetsRx = 0;
uint32_t octetsRx = 0;
uint32_t paquetsTx = 0;
uint32_t imagesCompletes = 0;
float ipsVideo = 0.0f;
int batteriePct = -1; // -1 si le drone ne remonte pas la tension par UDP

// Assemblage d'image JPEG (FFD8 ... FFD9) et envoi par tranches Base64.
uint8_t tamponRx[TAILLE_RX_UDP];
uint8_t tamponJpeg[TAILLE_MAX_JPEG];
size_t tailleJpeg = 0;
bool enCoursJpeg = false;
uint8_t octetPrecedent = 0;

uint8_t imagePrete[TAILLE_MAX_JPEG];
size_t tailleImagePrete = 0;
uint32_t seqImagePrete = 0;
uint32_t seqImageEnCours = 0;
size_t offsetEnvoiImage = 0;

WiFiUDP udpVideo;     // Port local 8800 (flux video UDP HD-DRONE / Lewei / Beken)
WiFiUDP udpControle;  // Port local 8888 (commandes et telemetrie UDP)
WiFiUDP udpAlt;       // Port local 50000 (variante UDP)
WiFiClient clientTcp; // Flux RTSP/HTTP eventuel (ports 7070, 554, 8080)
bool socketsOuverts = false;
bool tcpTente = false;

bool commenceParIgnoreCasse(const char *chaine, const char *prefixe) {
  return strncasecmp(chaine, prefixe, std::strlen(prefixe)) == 0;
}

bool contientIgnoreCasse(const char *chaine, const char *motif) {
  if (!chaine || !motif || !*motif) return false;
  const size_t n = std::strlen(motif);
  for (const char *p = chaine; *p; ++p) {
    if (strncasecmp(p, motif, n) == 0) return true;
  }
  return false;
}

bool estReseauDomestiqueIgnore(const char *ssid) {
  if (!ssid || !*ssid) return true;
  static const char *const IGNORES[] = {
    "SFR_", "Livebox", "Extender-WiFi", "SoundBarSetup",
    "ShellyPlusPlug", "Shelly", "Chromecast", "Bbox", "Freebox", "Orange"
  };
  for (const char *motif : IGNORES) {
    if (contientIgnoreCasse(ssid, motif)) return true;
  }
  return false;
}

bool ressembleADrone(const char *ssid, bool ouvert) {
  if (!ssid || !*ssid) return false;
  if (estReseauDomestiqueIgnore(ssid)) return false;
  static const char *const MOTIFS_DRONE[] = {
    "HD-", "HD_", "HDD", "WiFi-", "WIFI-", "WiFi_", "WIFI_", "Flow", "UFO",
    "HF-", "E88", "E800", "E99", "E58", "KY-", "KY_", "JX", "4D-", "4DRC",
    "FPV", "DRONE", "LW-", "GW-", "SG-", "S1", "KF-", "YX-", "SYMA",
    "JJRC", "LH-", "SH-", "CAM", "720P", "1080P", "4K", "8K", "UAV", "RC-"
  };
  for (const char *motif : MOTIFS_DRONE) {
    if (contientIgnoreCasse(ssid, motif)) return true;
  }
  // Tout autre reseau Wi-Fi ouvert inconnu a proximite immediate est un candidat drone.
  return ouvert;
}

static const char TABLE_B64[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t encoderBase64(const uint8_t *src, size_t n, char *dst, size_t cap) {
  size_t j = 0;
  for (size_t i = 0; i < n; i += 3) {
    if (j + 4 >= cap) break;
    const uint32_t v = (static_cast<uint32_t>(src[i]) << 16) |
                       (i + 1 < n ? (static_cast<uint32_t>(src[i + 1]) << 8) : 0) |
                       (i + 2 < n ? static_cast<uint32_t>(src[i + 2]) : 0);
    dst[j++] = TABLE_B64[(v >> 18) & 0x3F];
    dst[j++] = TABLE_B64[(v >> 12) & 0x3F];
    dst[j++] = (i + 1 < n) ? TABLE_B64[(v >> 6) & 0x3F] : '=';
    dst[j++] = (i + 2 < n) ? TABLE_B64[v & 0x3F] : '=';
  }
  dst[j] = '\0';
  return j;
}

void nettoyerChaine(const char *src, char *dst, size_t cap) {
  size_t j = 0;
  for (size_t i = 0; src && src[i] && j + 1 < cap; ++i) {
    char c = src[i];
    if (c == ' ' || c == ';' || c == '=' || c < 32 || c > 126) c = '_';
    dst[j++] = c;
  }
  dst[j] = '\0';
}

void publierEtat() {
  char ssidPropre[40] = "-";
  nettoyerChaine(ssidConnecte[0] ? ssidConnecte : ssidCible, ssidPropre, sizeof ssidPropre);
  if (!ssidPropre[0]) std::strcpy(ssidPropre, "-");
  console::printf(
      "[DRONE] etat=%s ssid=%s bssid=%s canal=%d rssi=%d ip=%s gw=%s "
      "proto=%s port_video=%u port_cmd=%u rx_pkts=%lu rx_kb=%lu tx_pkts=%lu "
      "ips=%.1f img=%lu bat=%d cmd=%s\n",
      etatCourant, ssidPropre, bssidConnecte[0] ? bssidConnecte : "-",
      canalWifi, rssiDbm, ipLocale[0] ? ipLocale : "-",
      ipPasserelle[0] ? ipPasserelle : "-", protocoleDetecte,
      static_cast<unsigned>(portVideo), static_cast<unsigned>(portCmd),
      static_cast<unsigned long>(paquetsRx),
      static_cast<unsigned long>(octetsRx / 1024),
      static_cast<unsigned long>(paquetsTx),
      ipsVideo, static_cast<unsigned long>(imagesCompletes),
      batteriePct, nomActionVol);
}

void ouvrirSockets() {
  if (socketsOuverts) return;
  udpVideo.begin(8800);
  udpControle.begin(8888);
  udpAlt.begin(50000);
  socketsOuverts = true;
  tcpTente = false;
}

void fermerSockets() {
  if (!socketsOuverts) return;
  udpVideo.stop();
  udpControle.stop();
  udpAlt.stop();
  if (clientTcp.connected()) clientTcp.stop();
  socketsOuverts = false;
  tcpTente = false;
}

void envoyerPaquetUdp(WiFiUDP &sock, IPAddress ip, uint16_t port, const uint8_t *donnees, size_t n) {
  if (sock.beginPacket(ip, port)) {
    sock.write(donnees, n);
    if (sock.endPacket()) paquetsTx++;
  }
}

// Reveille le flux video du drone sur les ports UDP/RTSP connus des drones HD-DRONE.
void sonderFluxVideo(IPAddress gw) {
  if (!gw || gw == IPAddress(0, 0, 0, 0)) return;
  // 1. Handshake UDP Lewei / HD-DRONE / Beken / Jianrong / FH8610
  static const uint8_t INIT_LEWEI[] = {0xef, 0x00, 0x04, 0x00};
  static const uint8_t INIT_BV[] = {0x42, 0x76}; // "Bv"
  static const uint8_t INIT_01[] = {0x01, 0x01};
  static const uint8_t INIT_CMD[] = {0xef, 0x02, 0x00, 0x00};
  for (uint16_t port : {8800, 8888, 50000, 7070, 8080}) {
    envoyerPaquetUdp(udpVideo, gw, port, INIT_LEWEI, sizeof INIT_LEWEI);
    envoyerPaquetUdp(udpVideo, gw, port, INIT_BV, sizeof INIT_BV);
    envoyerPaquetUdp(udpControle, gw, port, INIT_01, sizeof INIT_01);
  }
  envoyerPaquetUdp(udpControle, gw, 8888, INIT_CMD, sizeof INIT_CMD);

  // 2. Si aucun paquet UDP video n'est encore arrive, tester une ouverture TCP/RTSP (7070 / 554 / 8080)
  if (!tcpTente && paquetsRx == 0 && !clientTcp.connected()) {
    tcpTente = true;
    for (uint16_t port : {7070, 554, 8554, 8080, 80}) {
      if (clientTcp.connect(gw, port, 250)) {
        portVideo = port;
        if (port == 8080 || port == 80) {
          std::snprintf(protocoleDetecte, sizeof protocoleDetecte, "HTTP:%u", port);
          clientTcp.print("GET /?action=stream HTTP/1.1\r\nHost: ");
          clientTcp.print(gw.toString());
          clientTcp.print("\r\n\r\n");
        } else {
          std::snprintf(protocoleDetecte, sizeof protocoleDetecte, "RTSP:%u", port);
          char req[256];
          std::snprintf(req, sizeof req,
                        "SETUP rtsp://%s:%u/webcam RTSP/1.0\r\nCSeq: 1\r\n"
                        "Transport: RTP/AVP;unicast;client_port=8800-8801\r\n\r\n"
                        "PLAY rtsp://%s:%u/webcam RTSP/1.0\r\nCSeq: 2\r\n\r\n",
                        gw.toString().c_str(), port, gw.toString().c_str(), port);
          clientTcp.print(req);
        }
        break;
      }
    }
  }
}

// Envoie la trame de controle de vol 8 octets (protocole 0x66 ... 0x99 et 0xCC ... 0x33).
void envoyerCommandeVol(IPAddress gw, uint32_t maintenant) {
  if (!gw || gw == IPAddress(0, 0, 0, 0)) return;
  uint8_t flags = drapeauVol.load();
  if (flags && static_cast<int32_t>(maintenant - finDrapeauMs.load()) >= 0) {
    drapeauVol.store(0);
    flags = 0;
    if (!envoiVolActif.load()) nomActionVol = "aucune";
  }
  if (!flags && !envoiVolActif.load()) return;

  auto mapperAxe = [](int v) -> uint8_t {
    const int c = constrain(v, -100, 100);
    return static_cast<uint8_t>(constrain(128 + (c * 127) / 100, 0, 255));
  };
  const uint8_t roulis = mapperAxe(cmdRoulis.load());
  const uint8_t tangage = mapperAxe(cmdTangage.load());
  const uint8_t gaz = flags & 0x04 ? 0 : mapperAxe(cmdGaz.load());
  const uint8_t lacet = mapperAxe(cmdLacet.load());

  uint8_t trame66[8] = {0x66, roulis, tangage, gaz, lacet, flags, 0x00, 0x99};
  trame66[6] = trame66[1] ^ trame66[2] ^ trame66[3] ^ trame66[4] ^ trame66[5];
  for (uint16_t port : {8888, 8800, 50000, 7070}) {
    envoyerPaquetUdp(udpControle, gw, port, trame66, sizeof trame66);
  }
}

void inspecterOctets(const uint8_t *donnees, size_t n, uint16_t portSource, const char *proto) {
  if (!n) return;
  paquetsRx++;
  octetsRx += n;
  if (portVideo == 0 && portSource != 0) portVideo = portSource;
  if (std::strcmp(protocoleDetecte, "attente") == 0 && proto) {
    std::snprintf(protocoleDetecte, sizeof protocoleDetecte, "%s:%u", proto, portSource);
  }

  // Heuristique de telemetrie batterie sur petits paquets d'etat UDP (ex. 66 .. 99 ou ef ..)
  if (n >= 5 && n <= 32 && (donnees[0] == 0x66 || donnees[0] == 0xCC || donnees[0] == 0xEF)) {
    for (size_t i = 1; i + 1 < n; ++i) {
      if (donnees[i] >= 1 && donnees[i] <= 100 && donnees[i - 1] == 0xB0) {
        batteriePct = donnees[i];
      }
    }
  }

  // Extraction d'images JPEG par marqueurs SOI (FF D8) et EOI (FF D9).
  for (size_t i = 0; i < n; ++i) {
    const uint8_t b = donnees[i];
    if (!enCoursJpeg) {
      if (octetPrecedent == 0xFF && b == 0xD8) {
        enCoursJpeg = true;
        tamponJpeg[0] = 0xFF;
        tamponJpeg[1] = 0xD8;
        tailleJpeg = 2;
      }
    } else {
      if (tailleJpeg < TAILLE_MAX_JPEG) {
        tamponJpeg[tailleJpeg++] = b;
      } else {
        enCoursJpeg = false;
        tailleJpeg = 0;
      }
      if (octetPrecedent == 0xFF && b == 0xD9 && enCoursJpeg) {
        if (tailleJpeg >= 256) {
          imagesCompletes++;
          if (fluxActif.load() && seqImageEnCours == seqImagePrete) {
            std::memcpy(imagePrete, tamponJpeg, tailleJpeg);
            tailleImagePrete = tailleJpeg;
            seqImagePrete++;
          }
        }
        enCoursJpeg = false;
        tailleJpeg = 0;
      }
    }
    octetPrecedent = b;
  }
}

void lireSockets() {
  if (!socketsOuverts) return;
  WiFiUDP *liste[] = {&udpVideo, &udpControle, &udpAlt};
  for (WiFiUDP *sock : liste) {
    for (int k = 0; k < 8; ++k) {
      const int taille = sock->parsePacket();
      if (taille <= 0) break;
      const int lu = sock->read(tamponRx, min(static_cast<size_t>(taille), sizeof tamponRx));
      if (lu > 0) inspecterOctets(tamponRx, static_cast<size_t>(lu), sock->remotePort(), "UDP");
    }
  }
  if (clientTcp.connected()) {
    for (int k = 0; k < 8 && clientTcp.available() > 0; ++k) {
      const int lu = clientTcp.read(tamponRx, sizeof tamponRx);
      if (lu > 0) inspecterOctets(tamponRx, static_cast<size_t>(lu), portVideo, "TCP");
    }
  }
}

// Envoie progressivement l'image JPEG prete sur le port serie sans jamais engorger console::file.
void diffuserImageSerie() {
  if (!fluxActif.load()) return;
  if (console::espaceLibre() < 6144) return; // Priorite absolue a T et I !
  if (seqImageEnCours != seqImagePrete) {
    seqImageEnCours = seqImagePrete;
    offsetEnvoiImage = 0;
  }
  if (tailleImagePrete == 0 || offsetEnvoiImage >= tailleImagePrete) return;

  constexpr size_t OCTETS_PAR_TRANCHE = 180; // 180 octets -> 240 caracteres Base64 (< 320 octets/ligne)
  const size_t totalTranches = (tailleImagePrete + OCTETS_PAR_TRANCHE - 1) / OCTETS_PAR_TRANCHE;
  const size_t indiceTranche = offsetEnvoiImage / OCTETS_PAR_TRANCHE;
  const size_t n = min(OCTETS_PAR_TRANCHE, tailleImagePrete - offsetEnvoiImage);

  char b64[256];
  encoderBase64(imagePrete + offsetEnvoiImage, n, b64, sizeof b64);
  offsetEnvoiImage += n;

  console::printf("[DRONE_JPG] %lu %u/%u %s\n",
                  static_cast<unsigned long>(seqImageEnCours),
                  static_cast<unsigned>(indiceTranche + 1),
                  static_cast<unsigned>(totalTranches), b64);
}

void traiterResultatScan(int nbReseaux) {
  char resume[240] = "";
  size_t pos = 0;
  int meilleurIndice = -1;
  int meilleurRssi = -999;

  char cibleLocale[40] = "";
  char mdpLocal[40] = "";
  if (verrouConfig && xSemaphoreTake(verrouConfig, pdMS_TO_TICKS(20)) == pdTRUE) {
    std::strncpy(cibleLocale, ssidCible, sizeof cibleLocale - 1);
    std::strncpy(mdpLocal, motDePasseCible, sizeof mdpLocal - 1);
    xSemaphoreGive(verrouConfig);
  }

  for (int i = 0; i < nbReseaux; ++i) {
    String s = WiFi.SSID(i);
    const int rssi = WiFi.RSSI(i);
    const int ch = WiFi.channel(i);
    const bool ouvert = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
    char nomPropre[32];
    nettoyerChaine(s.c_str(), nomPropre, sizeof nomPropre);
    if (!nomPropre[0]) continue;

    const bool candidat = cibleLocale[0]
        ? (strcasecmp(s.c_str(), cibleLocale) == 0)
        : ressembleADrone(s.c_str(), ouvert);

    if (pos + 40 < sizeof resume && (ouvert || candidat || i < 6)) {
      pos += std::snprintf(resume + pos, sizeof resume - pos, "%s%s(%d,%s,c%d)",
                           pos ? ";" : "", nomPropre, rssi, ouvert ? "ouvert" : "wpa", ch);
    }
    if (candidat && rssi > meilleurRssi) {
      meilleurRssi = rssi;
      meilleurIndice = i;
    }
  }

  console::printf("[DRONE_SCAN] n=%d Candidat=%s reseaux=%s\n",
                  nbReseaux,
                  meilleurIndice >= 0 ? WiFi.SSID(meilleurIndice).c_str() : "aucun",
                  resume[0] ? resume : "aucun");

  if (meilleurIndice >= 0 && WiFi.status() != WL_CONNECTED) {
    String choisi = WiFi.SSID(meilleurIndice);
    std::strncpy(ssidConnecte, choisi.c_str(), sizeof ssidConnecte - 1);
    canalWifi = WiFi.channel(meilleurIndice);
    rssiDbm = WiFi.RSSI(meilleurIndice);
    etatCourant = "connexion";
    publierEtat();
    if (mdpLocal[0]) WiFi.begin(choisi.c_str(), mdpLocal);
    else WiFi.begin(choisi.c_str());
  }
  WiFi.scanDelete();
}

void tacheDrone(void *) {
  vTaskDelay(pdMS_TO_TICKS(1200)); // Laisser demarrer l'USB, les moteurs et l'I2C d'abord.
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  // Limiter la puissance d'emission RF pour eviter les appels de courant sur le 5V USB du Pi !
  WiFi.setTxPower(WIFI_POWER_11dBm);
  WiFi.disconnect(false, true);

  uint32_t dernierEtat = 0;
  uint32_t dernierScan = 0;
  uint32_t derniereSonde = 0;
  uint32_t dernierVol = 0;
  uint32_t debutConnexion = 0;
  uint32_t dernierCalculIps = millis();
  uint32_t imagesPrecedentes = 0;
  bool scanEnCours = false;

  for (;;) {
    const uint32_t maintenant = millis();

    // 1. Deconnexion ou reconnexion demandee par l'operateur
    if (demandeDeconnexion.exchange(false)) {
      fermerSockets();
      WiFi.disconnect(false, true);
      etatCourant = "inactif";
      ssidConnecte[0] = '\0';
      bssidConnecte[0] = '\0';
      ipLocale[0] = '\0';
      ipPasserelle[0] = '\0';
      std::strcpy(protocoleDetecte, "attente");
      portVideo = 0;
      rssiDbm = 0;
      ipsVideo = 0.0f;
      publierEtat();
    }

    if (demandeReconnexion.exchange(false)) {
      fermerSockets();
      WiFi.disconnect(false, true);
      char cible[40] = "", mdp[40] = "";
      if (verrouConfig && xSemaphoreTake(verrouConfig, pdMS_TO_TICKS(20)) == pdTRUE) {
        std::strncpy(cible, ssidCible, sizeof cible - 1);
        std::strncpy(mdp, motDePasseCible, sizeof mdp - 1);
        xSemaphoreGive(verrouConfig);
      }
      if (cible[0]) {
        std::strncpy(ssidConnecte, cible, sizeof ssidConnecte - 1);
        etatCourant = "connexion";
        debutConnexion = maintenant;
        if (mdp[0]) WiFi.begin(cible, mdp);
        else WiFi.begin(cible);
        publierEtat();
      } else {
        demandeScan.store(true);
      }
    }

    // 2. Suivi de la connexion Wi-Fi
    const wl_status_t statut = WiFi.status();
    if (statut == WL_CONNECTED) {
      if (std::strcmp(etatCourant, "connecte") != 0) {
        etatCourant = "connecte";
        std::strncpy(ssidConnecte, WiFi.SSID().c_str(), sizeof ssidConnecte - 1);
        std::strncpy(bssidConnecte, WiFi.BSSIDstr().c_str(), sizeof bssidConnecte - 1);
        std::strncpy(ipLocale, WiFi.localIP().toString().c_str(), sizeof ipLocale - 1);
        std::strncpy(ipPasserelle, WiFi.gatewayIP().toString().c_str(), sizeof ipPasserelle - 1);
        canalWifi = WiFi.channel();
        ouvrirSockets();
        sonderFluxVideo(WiFi.gatewayIP());
        derniereSonde = maintenant;
        publierEtat();
      }
      rssiDbm = WiFi.RSSI();
      canalWifi = WiFi.channel();
      ouvrirSockets();
      lireSockets();
      diffuserImageSerie();

      if (maintenant - derniereSonde >= PERIODE_SONDE_VIDEO_MS) {
        derniereSonde = maintenant;
        sonderFluxVideo(WiFi.gatewayIP());
      }
      if (maintenant - dernierVol >= PERIODE_CMD_VOL_MS) {
        dernierVol = maintenant;
        envoyerCommandeVol(WiFi.gatewayIP(), maintenant);
      }
    } else {
      if (socketsOuverts) fermerSockets();
      if (std::strcmp(etatCourant, "connexion") == 0) {
        if (maintenant - debutConnexion > 10000) {
          WiFi.disconnect(false, true);
          etatCourant = "scan";
          demandeScan.store(true);
        }
      } else if (std::strcmp(etatCourant, "inactif") != 0) {
        etatCourant = "scan";
      }

      // Scan Wi-Fi asynchrone pour detecter le drone sans bloquer
      if (scanEnCours) {
        const int res = WiFi.scanComplete();
        if (res >= 0) {
          scanEnCours = false;
          dernierScan = maintenant;
          traiterResultatScan(res);
          if ( std::strcmp(etatCourant, "connexion") == 0 ) debutConnexion = maintenant;
        } else if (res == WIFI_SCAN_FAILED) {
          scanEnCours = false;
          dernierScan = maintenant;
        }
      } else if (demandeScan.exchange(false) ||
                 (std::strcmp(etatCourant, "inactif") != 0 && maintenant - dernierScan >= PERIODE_SCAN_MS)) {
        WiFi.scanNetworks(true, false);
        scanEnCours = true;
        dernierScan = maintenant;
      }
    }

    // Calcul des images/s toutes les secondes
    if (maintenant - dernierCalculIps >= 1000) {
      const uint32_t dt = maintenant - dernierCalculIps;
      const uint32_t diff = imagesCompletes - imagesPrecedentes;
      ipsVideo = dt > 0 ? (diff * 1000.0f) / dt : 0.0f;
      imagesPrecedentes = imagesCompletes;
      dernierCalculIps = maintenant;
    }

    if (demandeEtat.exchange(false) || maintenant - dernierEtat >= PERIODE_ETAT_MS) {
      dernierEtat = maintenant;
      publierEtat();
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

} // namespace

bool demarrerTache() {
  static bool demarree = false;
  if (demarree) return true;
  verrouConfig = xSemaphoreCreateMutex();
  if (xTaskCreate(tacheDrone, "drone", 8192, nullptr, 1, nullptr) != pdPASS) {
    console::println("[DRONE] Impossible de creer la tache Wi-Fi drone.");
    return false;
  }
  demarree = true;
  return true;
}

bool traiterCommande(const char *texte) {
  if (!texte) return false;
  while (*texte == ' ' || *texte == '\t') ++texte;
  if (strncasecmp(texte, "drone", 5) != 0) return false;
  const char *suite = texte + 5;
  if (*suite != '\0' && *suite != ' ' && *suite != '\t') return false;
  while (*suite == ' ' || *suite == '\t') ++suite;

  if (*suite == '\0' || strcasecmp(suite, "etat") == 0) {
    demandeEtat.store(true);
    return true;
  }
  if (strcasecmp(suite, "scan") == 0) {
    demandeScan.store(true);
    demandeEtat.store(true);
    console::println("[DRONE] Balayage Wi-Fi lance.");
    return true;
  }
  if (strcasecmp(suite, "off") == 0 || strcasecmp(suite, "deconnecter") == 0) {
    demandeDeconnexion.store(true);
    return true;
  }
  if (strncasecmp(suite, "flux", 4) == 0) {
    const char *arg = suite + 4;
    while (*arg == ' ') ++arg;
    fluxActif.store(*arg != '0');
    demandeEtat.store(true);
    return true;
  }
  if (strncasecmp(suite, "ssid", 4) == 0) {
    const char *arg = suite + 4;
    while (*arg == ' ') ++arg;
    if (verrouConfig && xSemaphoreTake(verrouConfig, pdMS_TO_TICKS(20)) == pdTRUE) {
      if (*arg == '\0' || strcasecmp(arg, "auto") == 0) {
        ssidCible[0] = '\0';
        motDePasseCible[0] = '\0';
      } else {
        // Format optionnel : drone ssid NOM [MOT_DE_PASSE]
        char copie[80];
        std::strncpy(copie, arg, sizeof copie - 1);
        copie[sizeof copie - 1] = '\0';
        char *sep = std::strchr(copie, ' ');
        if (sep) {
          *sep = '\0';
          while (*++sep == ' ') {}
          std::strncpy(motDePasseCible, sep, sizeof motDePasseCible - 1);
        } else {
          motDePasseCible[0] = '\0';
        }
        std::strncpy(ssidCible, copie, sizeof ssidCible - 1);
      }
      xSemaphoreGive(verrouConfig);
    }
    demandeReconnexion.store(true);
    return true;
  }
  if (strncasecmp(suite, "cmd", 3) == 0) {
    const char *arg = suite + 3;
    while (*arg == ' ') ++arg;
    const uint32_t fin = millis() + 1000;
    if (strcasecmp(arg, "decoller") == 0) {
      drapeauVol.store(0x01); finDrapeauMs.store(fin); nomActionVol = "decoller";
    } else if (strcasecmp(arg, "atterrir") == 0) {
      drapeauVol.store(0x02); finDrapeauMs.store(fin); nomActionVol = "atterrir";
    } else if (strcasecmp(arg, "stop") == 0) {
      drapeauVol.store(0x04); finDrapeauMs.store(fin);
      cmdGaz.store(0); cmdLacet.store(0); cmdTangage.store(0); cmdRoulis.store(0);
      envoiVolActif.store(false); nomActionVol = "stop";
    } else if (strcasecmp(arg, "calibrer") == 0) {
      drapeauVol.store(0x08); finDrapeauMs.store(fin); nomActionVol = "calibrer";
    }
    demandeEtat.store(true);
    return true;
  }
  if (strncasecmp(suite, "manette", 7) == 0) {
    int g = 0, l = 0, t = 0, r = 0;
    if (std::sscanf(suite + 7, "%d %d %d %d", &g, &l, &t, &r) == 4) {
      cmdGaz.store(constrain(g, -100, 100));
      cmdLacet.store(constrain(l, -100, 100));
      cmdTangage.store(constrain(t, -100, 100));
      cmdRoulis.store(constrain(r, -100, 100));
      envoiVolActif.store(g != 0 || l != 0 || t != 0 || r != 0);
      nomActionVol = envoiVolActif.load() ? "manette" : "aucune";
    }
    return true;
  }
  demandeEtat.store(true);
  return true;
}

} // namespace drone
