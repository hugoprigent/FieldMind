#ifdef FIELDMIND_MOTOR_CONTROL

#include "moteur_zs_x11d1.h"
#include "moteur_policy.h"
#include "moteur_sorties.h"

#if defined(FIELDMIND_MOTOR_ONLY) || defined(FIELDMIND_DUAL_MOTOR)
#define FIELDMIND_MOTOR_CONSOLE
#endif
#include <Arduino.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>
#include <errno.h>
#include <stdlib.h>
#include <string.h>

namespace moteur {
namespace {
static_assert(PIN_PWM == ::D0 && PIN_DIR == ::D1 && PIN_FREIN == ::D10,
              "Verifier la correspondance Dx/GPIO pour la carte selectionnee");
#ifdef FIELDMIND_DUAL_MOTOR
static_assert(PINS_PWM[1] == ::D9, "Verifier D9 pour le deuxieme moteur");
#endif
constexpr uint32_t PWM_HZ = 5000;
constexpr uint8_t PWM_BITS = 10;
constexpr uint32_t PWM_MAX = (1U << PWM_BITS) - 1;
Policy controle;
SemaphoreHandle_t verrou = nullptr;
char jeton[9] = {};
char ligne[96] = {};
size_t taille = 0;
bool jeter = false;
bool pwmDisponible = false;
bool pwmAttache[NOMBRE_MOTEURS] = {};
std::atomic<bool> journalCapteurs{
#ifdef FIELDMIND_DUAL_MOTOR
    false
#else
    true
#endif
};
#ifdef FIELDMIND_DUAL_MOTOR
QueueHandle_t diagnosticsEnAttente = nullptr;
bool consoleAutonome = false;
#endif
uint32_t coupuresUsb = 0;
uint32_t dernierCycleMoteur = 0;
#ifdef FIELDMIND_MOTOR_CONSOLE
bool consoleConnectee = false;
const char *derniereRaison = "";
bool defautAnnonce = false;
#else
uint32_t dernierePublication = 0;
#endif

void detacherPwm() {
  for (size_t i = 0; i < NOMBRE_MOTEURS; ++i) {
    if (pwmAttache[i]) ledcDetach(PINS_PWM[i]);
    pwmAttache[i] = false;
    gpio_set_level(static_cast<gpio_num_t>(PINS_PWM[i]), 0);
    pinMode(PINS_PWM[i], OUTPUT);
  }
  pwmDisponible = false;
}

bool ecrirePwm(uint32_t valeur) {
  return ecrireGroupePwm(PINS_PWM, valeur, [](uint8_t pin, uint32_t duty) {
    return ledcWrite(pin, duty);
  });
}

// Une seule tâche applique une consigne commune à tout le groupe.
// Les écritures PWM sont successives dans le même cycle, pas une synchro de phase.
void tache(void *) {
  const TickType_t pause = pdMS_TO_TICKS(5) > 0 ? pdMS_TO_TICKS(5) : 1;
  uint32_t changementDirection = 0;
  bool attenteDirection = false;
  while (true) {
    if (xSemaphoreTake(verrou, portMAX_DELAY) == pdTRUE) {
      // Garder le motif d'un arret au repos et compter les commandes annulees.
      if (!Serial && controle.phase != Phase::Desarme) {
        controle.stop("usb_deconnecte");
        ++coupuresUsb;
      }
      const uint32_t maintenant = millis();
      dernierCycleMoteur = maintenant;
      uint8_t pourcent = controle.pwm(maintenant);
      static bool sensApplique = false, freinApplique = false;
      bool ok = pwmDisponible;
      // Couper le PWM avant tout changement de sens/frein.
      if (ok && (sensApplique != controle.direction || freinApplique != controle.frein)) {
        ok = ecrirePwm(0);
        if (ok && sensApplique != controle.direction) {
          changementDirection = maintenant;
          attenteDirection = true;
        }
        if (ok) {
          digitalWrite(PIN_DIR, controle.direction ? HIGH : LOW);
          digitalWrite(PIN_FREIN, controle.frein ? HIGH : LOW);
          sensApplique = controle.direction;
          freinApplique = controle.frein;
        }
      }
      if (attenteDirection) {
        if (maintenant - changementDirection < Policy::PREPARATION_MS) pourcent = 0;
        else attenteDirection = false;
      }
      if (ok) ok = ecrirePwm(static_cast<uint32_t>(pourcent) * PWM_MAX / 100);
      if (pwmDisponible && !ok) {
        controle.panne("ecriture_pwm");
        detacherPwm();
      }
      xSemaphoreGive(verrou);
    }
    vTaskDelay(pause);
  }
}

void stop(const char *raison) {
  if (!verrou) return;
  xSemaphoreTake(verrou, portMAX_DELAY);
  controle.stop(raison);
  xSemaphoreGive(verrou);
}

void aide() {
  Serial.println("Tape une commande puis appuie sur Entree :");
#ifdef FIELDMIND_DUAL_MOTOR
  Serial.println("  Un seul nombre commande LES DEUX moteurs au meme pourcentage.");
  Serial.println("  Sens DIR et frein BRAKE communs : aucun pilotage independant ici.");
  Serial.println("  capteurs 1 / capteurs 0 : afficher / masquer les mesures periodiques");
#endif
#ifdef FIELDMIND_MOTOR_CONSOLE
  Serial.println("  0 a 100  : pourcentage de PWM maintenu jusqu'a stop ou 0");
  Serial.println("             exemple : 50 = 50 % ; 0 = commande PWM nulle");
#endif
  Serial.println("  test     : essai tres court, 1 % de PWM pendant 200 ms");
  Serial.println("  test 2   : meme essai a 2 % (maximum 5 %)");
  Serial.println("  test -1  : essai dans l'autre sens, apres arret complet");
  Serial.println("  stop     : couper la commande PWM");
  Serial.println("  etat     : voir la commande actuelle");
#ifdef FIELDMIND_MOTOR_CONSOLE
  Serial.println("  diag     : verifier la configuration PWM et les coupures USB");
#endif
  Serial.println("  aide     : revoir ces instructions");
  Serial.println("  avance   : afficher les commandes avancees");
#ifndef FIELDMIND_MOTOR_ONLY
  Serial.println("  d / b    : diagnostics des capteurs (coupe la commande moteur)");
#endif
}

void etat() {
  if (!verrou) {
#ifdef FIELDMIND_MOTOR_CONSOLE
    Serial.println("[ERREUR] Initialisation moteur impossible. Aucun essai disponible. Redemarre l'ESP.");
#else
    Serial.println("{\"type\":\"motor\",\"etat\":\"DEFAUT\",\"raison\":\"initialisation\",\"pwm_pct\":0}");
#endif
    return;
  }
  xSemaphoreTake(verrou, portMAX_DELAY);
  Policy copie = controle;
  xSemaphoreGive(verrou);
  const uint32_t maintenant = millis();
  const uint8_t pourcent = copie.pwm(maintenant);
  const uint32_t restant = copie.phase == Phase::Impulsion || copie.phase == Phase::Preparation
      ? copie.finImpulsion - maintenant : 0;
#ifdef FIELDMIND_MOTOR_CONSOLE
  const char *description = "En attente d'une commande";
  if (copie.defaut) description = "Defaut logiciel : essais bloques. Redemarre l'ESP";
  else if (copie.frein) description = "Essais bloques par la commande de frein. Tape frein 0 pour la relacher";
  else if (copie.phase == Phase::Arme) description = "Arme, en attente d'une impulsion (expiration apres 5 s)";
  else if (copie.phase == Phase::Preparation) description = "Essai programme, preparation du sens";
  else if (copie.phase == Phase::Impulsion) description = "Commande d'essai en cours";
  else if (copie.phase == Phase::PreparationContinue) description = "Preparation de la commande continue";
  else if (copie.phase == Phase::Continu) description = "Commande maintenue jusqu'a stop ou 0, sans limite de duree";
  Serial.printf("[ETAT] %s.\n", description);
  Serial.printf("       Commande PWM : %u %% | Sens : %s (DIR=%u) | BRAKE=%u\n",
                pourcent, copie.direction ? "2" : "1", copie.direction ? 1 : 0, copie.frein ? 1 : 0);
#ifdef FIELDMIND_DUAL_MOTOR
  Serial.printf("       Moteur 1 (D0) : %u %% | Moteur 2 (D9) : %u %% demandes\n", pourcent, pourcent);
#endif
  if (restant) Serial.printf("       Fin de la commande dans au plus %lu ms.\n", static_cast<unsigned long>(restant));
#else
  Serial.printf("{\"type\":\"motor\",\"etat\":\"%s\",\"pwm_pct\":%u,\"dir\":%u,"
                "\"frein\":%s,\"restant_ms\":%lu,\"raison\":\"%s\",\"jeton\":\"%s\"}\n",
                copie.nom(), pourcent, copie.direction ? 1 : 0,
                copie.frein ? "true" : "false", static_cast<unsigned long>(restant), copie.raison, jeton);
#endif
}

#ifdef FIELDMIND_MOTOR_CONSOLE
void diagnostic() {
  etat();
  if (!verrou) return;
  xSemaphoreTake(verrou, portMAX_DELAY);
  const Policy copie = controle;
  const bool disponible = pwmDisponible;
  uint32_t registres[NOMBRE_MOTEURS] = {}, frequences[NOMBRE_MOTEURS] = {};
  for (size_t i = 0; i < NOMBRE_MOTEURS; ++i) {
    if (disponible) {
      registres[i] = ledcRead(PINS_PWM[i]);
      frequences[i] = ledcReadFreq(PINS_PWM[i]);
    }
  }
  const uint32_t coupures = coupuresUsb;
  const uint32_t cycle = dernierCycleMoteur;
  xSemaphoreGive(verrou);
#ifdef FIELDMIND_DUAL_MOTOR
  Serial.printf("[DIAG] Firmware main - deux moteurs, compile le %s a %s\n", __DATE__, __TIME__);
  Serial.println("       PWM1=D0/GPIO1 ; PWM2=D9/GPIO9 ; DIR et BRAKE communs.");
#else
  Serial.printf("[DIAG] Firmware test_moteur compile le %s a %s\n", __DATE__, __TIME__);
#endif
  Serial.println("       PWM=D0/GPIO1 ; DIR=D1/GPIO0 ; BRAKE=D10/GPIO10.");
  Serial.printf("       Generateur PWM : %s\n", disponible ? "initialise" : "INDISPONIBLE");
  for (size_t i = 0; i < NOMBRE_MOTEURS; ++i) {
    Serial.printf("       Moteur %u / GPIO%u : registre %.1f %% ; frequence %lu Hz\n",
                  static_cast<unsigned>(i + 1), PINS_PWM[i],
                  100.0 * registres[i] / (PWM_MAX + 1U), static_cast<unsigned long>(frequences[i]));
  }
  Serial.println("       Attendu : 5000 Hz si PWM non nul ; frequence non relue a 0 %.");
  Serial.printf("       Dernier passage de la tache moteur : il y a %lu ms.\n",
                static_cast<unsigned long>(millis() - cycle));
  Serial.printf("       Commandes annulees par detection USB : %lu\n", static_cast<unsigned long>(coupures));
  if (!strcmp(copie.raison, "usb_deconnecte"))
    Serial.println("       Motif : connexion USB detectee comme perdue ; commande annulee.");
  else Serial.printf("       Motif logiciel : %s\n", copie.raison);
  Serial.println("       Ces lectures verifient la configuration ESP, pas la tension sur le fil.");
  Serial.println("       Mesure au multimetre : batterie debranchee, commandes retirees des drivers.");
  Serial.println("       Noir sur GND ESP, rouge sur une sortie PWM : 0 -> 0 V ; 100 -> environ 3,3 V.");
  Serial.println("       Terminer par stop avant de rebrancher quoi que ce soit.");
}

// La console annonce les evenements une seule fois ; la tache moteur ne fait
// aucune ecriture serie pour ne pas retarder la coupure du PWM.
void annoncerEvenement() {
  if (!verrou) return;
  xSemaphoreTake(verrou, portMAX_DELAY);
  const Policy copie = controle;
  xSemaphoreGive(verrou);
  if (copie.defaut && !defautAnnonce) {
    Serial.println("[ERREUR] Defaut de commande moteur. Essais bloques. Redemarre l'ESP.");
    defautAnnonce = true;
  }
  if (!strcmp(derniereRaison, copie.raison)) return;
  derniereRaison = copie.raison;
  if (!strcmp(copie.raison, "impulsion_terminee")) {
    Serial.println("[FIN] Duree terminee. Commande PWM demandee : 0 %.");
    Serial.println("      Nouvelle commande : un nombre de 0 a 100 puis Entree.");
    Serial.println("      Si ca vibre ou reste bloque : tape stop et verifie le cablage.");
  } else if (!strcmp(copie.raison, "armement_expire")) {
    Serial.println("[ATTENTE] Armement expire sans impulsion. Commande PWM a 0 %.");
  } else if (!strcmp(copie.raison, "usb_deconnecte")) {
    Serial.println("[COUPURE] Connexion USB detectee comme perdue ; commande annulee. Tape diag.");
  }
}
#endif

bool entier(const char *texte, long &valeur) {
  if (!texte || !*texte) return false;
  errno = 0;
  char *fin = nullptr;
  valeur = strtol(texte, &fin, 10);
  return errno == 0 && fin && !*fin;
}

char commande(char *texte) {
  char *args[5] = {};
  char *suite = nullptr;
  size_t n = 0;
  for (char *arg = strtok_r(texte, " \t", &suite); arg; arg = strtok_r(nullptr, " \t", &suite)) {
    if (n == 5) { stop("commande_invalide"); Serial.println("ERREUR trop de parametres"); return 0; }
    args[n++] = arg;
  }
  if (!n) return 0;
#ifdef FIELDMIND_MOTOR_CONSOLE
  long valeur = 0;
  if (n == 1 && entier(args[0], valeur)) {
    if (valeur < 0 || valeur > 100) {
      stop("commande_invalide");
      Serial.println("[ERREUR] Choisir un entier de 0 a 100. Commande PWM demandee : 0 %.");
      return 0;
    }
    if (!verrou) { etat(); return 0; }
    xSemaphoreTake(verrou, portMAX_DELAY);
    const bool ok = controle.consigne(static_cast<int>(valeur), millis());
    const bool sens = controle.direction;
    xSemaphoreGive(verrou);
    if (!ok) {
      Serial.println("[REFUS] Commande bloquee par un frein ou un defaut. Tape etat.");
      etat();
    } else if (!valeur) {
      Serial.println("[STOP] Commande PWM demandee : 0 %.");
    } else {
      Serial.printf("[PWM] %ld %% demandes sur %u moteur(s) | sens %u | maintien jusqu'a stop ou 0.\n",
                    valeur, static_cast<unsigned>(NOMBRE_MOTEURS), sens ? 2 : 1);
      Serial.println("      Pour couper la commande : 0 ou stop puis Entree.");
    }
    return 0;
  }
#endif
  if (n == 1 && !strcmp(args[0], "aide")) {
    aide();
#ifdef FIELDMIND_DUAL_MOTOR
  } else if (n == 2 && !strcmp(args[0], "capteurs") &&
             (!strcmp(args[1], "0") || !strcmp(args[1], "1"))) {
    journalCapteurs = args[1][0] == '1';
    Serial.println(journalCapteurs ? "[CAPTEURS] Mesures periodiques affichees." : "[CAPTEURS] Mesures periodiques masquees ; capteurs toujours actifs.");
#endif
  } else if (n == 1 && !strcmp(args[0], "avance")) {
    Serial.printf("  armer %s : autoriser une seule impulsion, valable 5 s\n", jeton);
    Serial.println("  impulsion <pourcent> <ms> : apres armement, +/-1 a 5 %, 50 a 500 ms");
    Serial.println("  frein 1 / frein 0 : PWM nul, puis niveau BRAKE haut / bas");
    Serial.println("  Le niveau BRAKE demande ne confirme pas le freinage physique.");
  } else if (n == 1 && !strcmp(args[0], "etat")) {
    etat();
#ifdef FIELDMIND_MOTOR_CONSOLE
  } else if (n == 1 && !strcmp(args[0], "diag")) {
    diagnostic();
#endif
  } else if (n == 1 && (!strcmp(args[0], "stop") || !strcmp(args[0], "desarmer"))) {
    stop("stop_operateur");
    Serial.println("[STOP] Commande PWM demandee : 0 %. L'arret physique n'est pas mesure.");
  } else if (n == 1 && (!strcmp(args[0], "d") || !strcmp(args[0], "b"))) {
    stop("diagnostic_capteurs");
#ifdef FIELDMIND_MOTOR_ONLY
    Serial.println("[INFO] Mode moteur seul : diagnostics capteurs desactives. Tape aide.");
#else
    return args[0][0];
#endif
  } else if ((n == 1 || n == 2) && !strcmp(args[0], "test") && verrou) {
    long pourcent = 1;
    if ((n == 2 && !entier(args[1], pourcent)) || pourcent == 0 ||
        pourcent < -Policy::POURCENT_MAX || pourcent > Policy::POURCENT_MAX) {
      stop("commande_invalide");
      Serial.println("ERREUR test : choisir 1 a 5 %, ou -1 a -5 % pour l'autre sens");
      return 0;
    }
    xSemaphoreTake(verrou, portMAX_DELAY);
    const bool ok = controle.essai(static_cast<int>(pourcent), millis());
    xSemaphoreGive(verrou);
    if (ok) Serial.printf("[ESSAI] Commande %ld %% PWM, sens %u, pendant %lu ms. Coupure automatique.\n",
                          pourcent < 0 ? -pourcent : pourcent, pourcent < 0 ? 2 : 1,
                          static_cast<unsigned long>(Policy::TEST_MS));
    else {
      Serial.println("[REFUS] Essai refuse. Commande PWM a 0 %. Tape etat pour voir pourquoi.");
      etat();
    }
  } else if (n == 2 && !strcmp(args[0], "armer") && !strcmp(args[1], jeton) && verrou) {
    xSemaphoreTake(verrou, portMAX_DELAY);
    const bool ok = controle.armer(millis());
    if (!ok) controle.stop("armement_refuse");
    xSemaphoreGive(verrou);
    Serial.println(ok ? "OK arme pour une seule impulsion" : "ERREUR armement refuse (frein, defaut ou deja arme)");
  } else if (n == 3 && !strcmp(args[0], "impulsion") && verrou) {
    long pourcent = 0, duree = 0;
    if (!entier(args[1], pourcent) || !entier(args[2], duree) ||
        pourcent < -Policy::POURCENT_MAX || pourcent > Policy::POURCENT_MAX ||
        duree < Policy::DUREE_MIN_MS || duree > Policy::DUREE_MAX_MS) {
      stop("commande_invalide");
      Serial.println("ERREUR limites : PWM signe 1..5 %, duree 50..500 ms");
      return 0;
    }
    xSemaphoreTake(verrou, portMAX_DELAY);
    const bool ok = controle.impulsion(static_cast<int>(pourcent), static_cast<uint32_t>(duree), millis());
    xSemaphoreGive(verrou);
    Serial.println(ok ? "OK impulsion programmee" : "ERREUR impulsion refusee, armer d'abord");
  } else if (n == 2 && !strcmp(args[0], "frein") &&
             (!strcmp(args[1], "0") || !strcmp(args[1], "1")) && verrou) {
    xSemaphoreTake(verrou, portMAX_DELAY);
    controle.reglerFrein(args[1][0] == '1');
    xSemaphoreGive(verrou);
    Serial.printf("[FREIN] Commande PWM a 0 % ; BRAKE=%c. %s\n", args[1][0],
                  args[1][0] == '1' ? "Essais bloques. Tape frein 0 pour les autoriser." : "Essais autorises hors defaut.");
  } else {
    stop("commande_invalide");
    Serial.println("ERREUR commande rejetee, moteur desarme ; taper aide");
  }
  return 0;
}
}  // namespace

bool journalCapteursActif() { return journalCapteurs; }

void demarrer() {
  // Pas d'attente USB avant de fixer les niveaux de repos.
  for (uint8_t pin : PINS_PWM) {
    gpio_set_level(static_cast<gpio_num_t>(pin), 0); pinMode(pin, OUTPUT);
  }
  gpio_set_level(static_cast<gpio_num_t>(PIN_DIR), 0); pinMode(PIN_DIR, OUTPUT);
  gpio_set_level(static_cast<gpio_num_t>(PIN_FREIN), 0); pinMode(PIN_FREIN, OUTPUT);
  snprintf(jeton, sizeof(jeton), "%08lx", static_cast<unsigned long>(esp_random()));
  verrou = xSemaphoreCreateMutex();
  if (!verrou) { controle.panne("mutex"); return; }
  for (size_t i = 0; i < NOMBRE_MOTEURS; ++i) {
    if (!ledcAttach(PINS_PWM[i], PWM_HZ, PWM_BITS)) {
      controle.panne("initialisation_pwm");
      detacherPwm();
      return;
    }
    pwmAttache[i] = true;
  }
  pwmDisponible = true;
  if (!ecrirePwm(0)) { controle.panne("ecriture_pwm"); detacherPwm(); return; }
  if (xTaskCreate(tache, "controle_moteur", 4096, nullptr, 2, nullptr) != pdPASS) {
    controle.panne("tache_moteur");
    detacherPwm();
  }
}

void annoncer() {
#ifdef FIELDMIND_MOTOR_CONSOLE
  if (!Serial) return; // L'accueil sera affiche quand le moniteur s'ouvrira.
  consoleConnectee = true;
  Serial.println();
#ifdef FIELDMIND_DUAL_MOTOR
  Serial.println("=== MAIN ROVER - DEUX MOTEURS ZS-X11D1 ===");
  Serial.println("PWM1=D0/GPIO1 ; PWM2=D9/GPIO9 ; DIR=D1/GPIO0 et BRAKE=D10/GPIO10 communs.");
  Serial.println("Ecran et capteurs actifs ; mesures periodiques masquees par defaut.");
#else
  Serial.println("=== TEST MOTEUR SEUL - ZS-X11D1 / ESP32-C5 ===");
  Serial.println("Aucun demarrage automatique. Pas de messages des capteurs.");
#endif
  Serial.println("Le PWM est une commande : la vitesse et le mouvement ne sont pas mesures.");
  Serial.println("0 % demande n'est pas un arret valide du driver : verifier l'entree utilisee.");
  aide();
  etat();
  Serial.println("PRET : un entier de 0 a 100 puis Entree ; maintien jusqu'a stop ou 0.");
#else
  Serial.println("=== FieldMind ESP C5 : capteurs + moteur ZS-X11D1 ===");
  Serial.println("D0/GPIO1=PWM 5kHz ; D1/GPIO0=DIR ; D10/GPIO10=BRAKE ; STOP non commande.");
  Serial.println("Demarrage DESARME. Aucune rotation automatique. Taper aide puis Entree.");
  Serial.println("Commande simple : test = 1 % de PWM pendant 200 ms ; stop = coupure PWM.");
  etat();
#endif
}

char lireConsoleInterne() {
#ifdef FIELDMIND_MOTOR_CONSOLE
  if (!Serial) {
    consoleConnectee = false;
    taille = 0;
    jeter = false;
    return 0;
  }
  if (!consoleConnectee) annoncer();
  annoncerEvenement();
#endif
  for (size_t lus = 0; lus < 128 && Serial.available(); ++lus) {
    const int ch = Serial.read();
    if (ch == '\r' || ch == '\n') {
      if (jeter) { jeter = false; taille = 0; continue; }
      ligne[taille] = 0;
      taille = 0;
      const char diagnostic = commande(ligne);
      if (diagnostic) return diagnostic;
#ifdef FIELDMIND_MOTOR_CONSOLE
      annoncerEvenement();
#endif
    } else if (ch == '\b' || ch == 127) {
      if (taille && !jeter) --taille;
    } else if (!jeter) {
      if (ch < 32 || ch > 126 || taille >= sizeof(ligne) - 1) {
        jeter = true;
        taille = 0;
        stop("ligne_invalide");
        Serial.println("ERREUR ligne jetee jusqu'au prochain saut de ligne ; desarme");
      } else ligne[taille++] = static_cast<char>(ch);
    }
  }
#ifndef FIELDMIND_MOTOR_CONSOLE
  if (millis() - dernierePublication >= 1000) {
    dernierePublication = millis();
    etat();
  }
#endif
  return 0;
}

#ifdef FIELDMIND_DUAL_MOTOR
void tacheConsole(void *) {
  const TickType_t pause = pdMS_TO_TICKS(1) > 0 ? pdMS_TO_TICKS(1) : 1;
  while (true) {
    const char diagnostic = lireConsoleInterne();
    if (diagnostic && xQueueSend(diagnosticsEnAttente, &diagnostic, 0) != pdTRUE)
      Serial.println("[CAPTEURS] File de diagnostics pleine ; commande moteur deja coupee.");
    vTaskDelay(pause);
  }
}

bool demarrerConsoleAutonome() {
  if (consoleAutonome) return true;
  if (!verrou) return false;
  diagnosticsEnAttente = xQueueCreate(4, sizeof(char));
  if (!diagnosticsEnAttente ||
      xTaskCreate(tacheConsole, "console_moteurs", 6144, nullptr, 1, nullptr) != pdPASS) {
    if (diagnosticsEnAttente) vQueueDelete(diagnosticsEnAttente);
    diagnosticsEnAttente = nullptr;
    xSemaphoreTake(verrou, portMAX_DELAY);
    controle.panne("tache_console");
    xSemaphoreGive(verrou);
    Serial.println("[ERREUR] Console autonome indisponible ; commandes moteur bloquees.");
    return false;
  }
  consoleAutonome = true;
  return true;
}
#endif

char lireConsole() {
#ifdef FIELDMIND_DUAL_MOTOR
  if (consoleAutonome) {
    char diagnostic = 0;
    if (xQueueReceive(diagnosticsEnAttente, &diagnostic, 0) == pdTRUE) return diagnostic;
    return 0;
  }
#endif
  return lireConsoleInterne();
}
}  // namespace moteur
#endif
