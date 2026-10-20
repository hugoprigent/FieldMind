// Execute le controleur reel, avec USB, horloge, GPIO et LEDC simules.
#include <Arduino.h>
#include "../src/test_tout_moteur_pwm.h"
#include "../src/actionneurs.h"
#include <Preferences.h>
#include <cstdlib>
#include <cstring>
#ifdef FIELDMIND_ROVER_COMPLET
#define VERSION_ATTENDUE "[FIRMWARE] ROVER_3"
#else
#define VERSION_ATTENDUE "[FIRMWARE] TOUT_MOTEUR_4"
#endif
uint32_t horloge=0;
uint32_t decalageMicros=0;
int modes[32]={};
void (*interruptions[32])()={};
bool creationTache=true;
bool memoireEcriture=true;
int niveaux[32]={};
Console Serial;
struct Sortie { bool active=false; ledc_channel_handle_t bus={}; uint32_t duty=0; } sorties[32];
bool ledcAttachChannel(uint8_t p, uint32_t hz, uint8_t bits, uint8_t canal) {
  sorties[p]={true,{p,canal,bits,static_cast<uint8_t>(canal/2),hz},0}; return true;
}
bool ledcWrite(uint8_t p, uint32_t duty) {
  if(!sorties[p].active) return false;
  sorties[p].duty=duty; return true;
}
bool ledcDetach(uint8_t p) { sorties[p].active=false; return true; }
uint32_t ledcRead(uint8_t p) { return sorties[p].duty; }
void *perimanGetPinBus(uint8_t p, peripheral_bus_type_t) {
  return sorties[p].active ? &sorties[p].bus : nullptr;
}
uint32_t ledc_get_freq(ledc_mode_t, ledc_timer_t timer) {
  for (const auto &s : sorties) if(s.active && s.bus.timer_num==timer) return s.bus.freq_hz;
  return 0;
}

// ---------------------------------------------------------------- ROVER_3
static int nr=0;
#define CHECK3(c) do { ++nr; if(!(c)) { std::fprintf(stderr,"ECHEC rover3 %d : %s\n",__LINE__,#c); return 1; } } while(0)
static void envoyer(const char *ligne) { Serial.entree=std::string(ligne)+"\n"; actionneurs::actualiser(); }
static void jusqua(uint32_t t) { while(horloge<=t) actionneurs::actualiser(); } // tours jusqu'a t inclus
static size_t compter(const std::string &texte, const char *motif) {
  size_t total=0;
  for(size_t i=texte.find(motif); i!=std::string::npos; i=texte.find(motif,i+1)) ++total;
  return total;
}
struct LigneT { unsigned long ms=0; int g=0, d=0; float servo=0; unsigned frein=9, defaut=9, chien=9; };
static int champsDerniereT() {
  const size_t i=Serial.sortie.rfind("\nT ");
  if(i==std::string::npos) return 0;
  const size_t fin=Serial.sortie.find('\n',i+1);
  const std::string ligne=Serial.sortie.substr(i+1,fin==std::string::npos?std::string::npos:fin-i-1);
  int n=0; bool dans=false;
  for(char c:ligne) { const bool espace=c==' '; if(!espace&&!dans) ++n; dans=!espace; }
  return n;
}
static bool derniereT(LigneT &t) { // derniere ligne T complete
  const size_t i=Serial.sortie.rfind("\nT ");
  if(i==std::string::npos) return false;
  return std::sscanf(Serial.sortie.c_str()+i+1,"T %lu %d %d %f %u %u %u\n",
                     &t.ms,&t.g,&t.d,&t.servo,&t.frein,&t.defaut,&t.chien)==7;
}

static int scenarioRover3() {
  modes[D2]=modes[D3]=-1; // sentinelle : roues 0 ne touche pas D2/D3
  actionneurs::demarrer();
  actionneurs::actualiser();
  CHECK3(Serial.sortie.find(VERSION_ATTENDUE "\n")!=std::string::npos);
  CHECK3(Serial.sortie.find("p 12 -8 5.5 :")!=std::string::npos && Serial.sortie.find("wd 400")!=std::string::npos);
#ifdef FIELDMIND_ROVER_COMPLET
  CHECK3(Serial.sortie.find("[PILOTAGE] Chien=0 ms (coupe) ; inv=500 ms ; tel=0 Hz ; imu=0 Hz ; lignes perdues=0\n")!=std::string::npos);
#else
  CHECK3(Serial.sortie.find("[PILOTAGE] Chien=0 ms (coupe) ; inv=500 ms ; tel=0 Hz ; imu=absente")!=std::string::npos);
#endif
  CHECK3(actionneurs::cadenceCentrale()==0);
  // p : silencieux, deux roues, sans chien de garde (comportement ROVER_2 : pas d'expiration).
  Serial.sortie.clear();
  envoyer("p 10 10 0");
  CHECK3(Serial.sortie.empty());
  jusqua(horloge+100);
  CHECK3(sorties[D0].duty==102 && sorties[D9].duty==102 && sorties[D7].active);
  CHECK3(niveaux[D1]==1 && niveaux[D4]==0);
  jusqua(horloge+3000);
  CHECK3(sorties[D0].duty==102 && sorties[D9].duty==102);
  CHECK3(!actionneurs::tractionAuRepos(horloge,1000));

  // Chien de garde : renouvele par hb et p seulement.
  Serial.sortie.clear();
  envoyer("wd 400");
  CHECK3(Serial.sortie=="[CHIEN] Delai 400 ms ; renouvele par p ou hb, sinon traction a zero.\n");
  for(int i=0;i<4;++i) { jusqua(horloge+300); envoyer("hb"); }
  for(int i=0;i<4;++i) { jusqua(horloge+350); envoyer("p 10 10 0"); }
  CHECK3(sorties[D0].duty==102 && compter(Serial.sortie,"[CHIEN]")==1);
  const uint32_t renouvele=horloge-1;
  jusqua(renouvele+150); envoyer("etat");
  jusqua(renouvele+250); envoyer("v 30");
  jusqua(renouvele+300); envoyer("m 10");     // m/g/d ne renouvellent pas le chien
  jusqua(renouvele+399);
  CHECK3(sorties[D0].duty==102 && compter(Serial.sortie,"[CHIEN] expire")==0);
  jusqua(renouvele+400);
  CHECK3(sorties[D0].duty==0 && sorties[D9].duty==0);
  CHECK3(sorties[D7].active);                 // servo maintenu
  CHECK3(compter(Serial.sortie,"[CHIEN] expire : traction a zero, servo maintenu ; envoyer p pour reprendre.\n")==1);
  jusqua(horloge+2000);
  CHECK3(compter(Serial.sortie,"[CHIEN] expire")==1 && sorties[D0].duty==0);
  Serial.sortie.clear();
  envoyer("m 10");                            // refuse tant que le chien est echu
  CHECK3(Serial.sortie=="[CHIEN] Delai depasse : m, g ou d refuse ; envoyer hb ou p.\n");
  jusqua(horloge+200);
  CHECK3(sorties[D0].duty==0 && sorties[D9].duty==0);
  envoyer("hb");                              // pas de reprise automatique
  jusqua(horloge+200);
  CHECK3(sorties[D0].duty==0 && sorties[D9].duty==0);
  const uint32_t dernierP=horloge;
  envoyer("p 10 10 0");
  jusqua(horloge+100);
  CHECK3(sorties[D0].duty==102 && sorties[D9].duty==102);
  // Une ligne invalide ne renouvelle pas (et coupe tout, comme en ROVER_2).
  jusqua(dernierP+300);
  Serial.sortie.clear();
  envoyer("p 10 10");
  CHECK3(Serial.sortie.find("[ERREUR] Commande invalide.")==0 && sorties[D0].duty==0 && !sorties[D7].active);
  jusqua(dernierP+399);
  CHECK3(compter(Serial.sortie,"[CHIEN] expire")==0);
  jusqua(dernierP+400);
  CHECK3(compter(Serial.sortie,"[CHIEN] expire")==1);
  Serial.sortie.clear();
  envoyer("wd 0");
  CHECK3(Serial.sortie=="[CHIEN] Coupe.\n");

  // p rattache le servo coupe et suit la cible ; meme cible : trajectoire non relancee.
  envoyer("p 0 0 20");
  CHECK3(sorties[D7].active);
  jusqua(horloge+2000);
  CHECK3(sorties[D7].duty==1500+156);         // 20 deg sur 90 = 155,6 us
  envoyer("p 0 0 0");
  jusqua(horloge+100);
  const uint32_t mi=sorties[D7].duty;
  envoyer("p 0 0 0");
  jusqua(horloge+100);
  CHECK3(sorties[D7].duty<mi && sorties[D7].duty>1500);
  jusqua(horloge+2000);
  CHECK3(sorties[D7].duty==1500);
  CHECK3(actionneurs::tractionAuRepos(horloge,1000));

  // Inversion : passage par zero, attente inv roue a zero, puis D/R et valeur ; droite intacte.
  envoyer("p 10 10 0");
  jusqua(horloge+100);
  CHECK3(sorties[D0].duty==102 && sorties[D9].duty==102 && niveaux[D1]==1 && niveaux[D4]==0);
  envoyer("tel 20");
  const uint32_t zero=horloge;
  envoyer("p -10 10 0");
  CHECK3(sorties[D0].duty==0 && sorties[D9].duty==102 && niveaux[D1]==1);
  for(uint32_t t=zero+50;t<zero+500;t+=50) { jusqua(t-1); envoyer("p -10 10 0"); } // comme le Pi a 20 Hz
  jusqua(zero+499);
  CHECK3(sorties[D0].duty==0 && niveaux[D1]==1 && sorties[D9].duty==102);
  LigneT t;
  CHECK3(derniereT(t) && t.g==0 && t.d==10 && t.frein==0 && t.defaut==0 && t.chien==0);
  jusqua(zero+500);
  CHECK3(niveaux[D1]==0 && sorties[D0].duty==0 && niveaux[D4]==0); // D/R change, PWM encore nul
  jusqua(zero+599);
  CHECK3(sorties[D0].duty==0);
  jusqua(zero+600);
  CHECK3(sorties[D0].duty==102 && sorties[D9].duty==102);
  jusqua(zero+700);
  CHECK3(derniereT(t) && t.g==-10 && t.d==10);
  // inv reglable ; une commande ROVER_2 de la roue annule l'inversion en attente.
  Serial.sortie.clear();
  envoyer("inv 200");
  CHECK3(Serial.sortie=="[PILOTAGE] Inversion : roue a zero 200 ms avant changement de D/R.\n");
  envoyer("p -10 20 0");
  jusqua(horloge+100);
  CHECK3(sorties[D9].duty==204);
  const uint32_t zeroD=horloge;
  envoyer("p -10 -20 0");
  CHECK3(sorties[D9].duty==0 && sorties[D0].duty==102);
  envoyer("d 0");
  jusqua(zeroD+1000);
  CHECK3(sorties[D9].duty==0 && niveaux[D4]==0);  // pas de reprise ni d'inversion
  envoyer("p 0 0 0");

  // Telemetrie : format, cadence, etat periodique suspendu.
  envoyer("wd 400");
  Serial.sortie.clear();
  const uint32_t debutT=horloge;
  for(uint32_t ti=debutT+50; ti<=debutT+2000; ti+=50) { jusqua(ti-1); envoyer("hb"); }
  jusqua(debutT+1999);
  const size_t nT=compter(Serial.sortie,"\nT ")+(Serial.sortie.rfind("T ",0)==0);
  CHECK3(nT>=39 && nT<=41);
  CHECK3(compter(Serial.sortie,"[FIRMWARE]")==0);
  CHECK3(derniereT(t) && t.g==0 && t.d==0 && t.chien==1 && t.servo==0.0f);
  Serial.sortie.clear();
  envoyer("tel 0");
#ifdef FIELDMIND_ROVER_COMPLET
  CHECK3(Serial.sortie.find("[PILOTAGE] Telemetrie coupee.\n")==0);
  jusqua(horloge+1100);
  CHECK3(compter(Serial.sortie,"[FIRMWARE] ROVER_3")>=1 && compter(Serial.sortie,"\nT ")==0);
#else
  CHECK3(Serial.sortie=="[PILOTAGE] Telemetrie coupee.\n");
#endif
  envoyer("wd 0");

  // Frein et course : refus comme m / s.
  envoyer("frein 1");
  Serial.sortie.clear();
  envoyer("p 0 0 0");
  CHECK3(Serial.sortie.empty() && sorties[D7].active);
  envoyer("p 10 0 0");
  CHECK3(Serial.sortie.find("[ERREUR] Commande moteur refusee : verifier frein et etat.")==0);
  CHECK3(sorties[D0].duty==0 && !sorties[D7].active);
  envoyer("frein 0");
  envoyer("course 140");
  Serial.sortie.clear();
  envoyer("p 0 0 80");
  CHECK3(Serial.sortie.find("[ERREUR] Angle hors de la course actuelle.")==0);

  // Sorties SC (roues) : D2/D3 non touchees et T a 8 champs tant que roues 0.
  CHECK3(modes[D2]==-1 && modes[D3]==-1 && !interruptions[D2] && !interruptions[D3]);
  envoyer("tel 20");
  jusqua(horloge+60);
  CHECK3(champsDerniereT()==8);
  Serial.sortie.clear();
  envoyer("roues 1");
  CHECK3(Serial.sortie.rfind("[ROUES] Lecture SC active : gauche D2/GPIO25, droite D3/GPIO7, fronts montants ; "
                             "T + hz_g hz_d ; compteurs a zero.\n",0)==0);
  CHECK3(modes[D2]==INPUT && modes[D3]==INPUT && interruptions[D2] && interruptions[D3]);
  envoyer("p 10 -10 0");                        // droite en arriere : frequence negative
  const uint32_t debutSC=horloge;
  bool rebond=false;
  while(horloge<debutSC+500) {                  // gauche 200 imp/s, droite 100 imp/s
    actionneurs::actualiser();
    if(horloge%5==0) {
      interruptions[D2]();
      if(!rebond && horloge>=debutSC+250) {     // rebond 50 us apres un front accepte
        decalageMicros=50; interruptions[D2](); decalageMicros=0; rebond=true;
      }
    }
    if(horloge%10==0) interruptions[D3]();
  }
  CHECK3(champsDerniereT()==10);
  LigneT t10;
  float hzG=0, hzD=0;
  {
    const size_t i=Serial.sortie.rfind("\nT ");
    CHECK3(std::sscanf(Serial.sortie.c_str()+i+1,"T %lu %d %d %f %u %u %u %f %f",&t10.ms,&t10.g,&t10.d,
                       &t10.servo,&t10.frein,&t10.defaut,&t10.chien,&hzG,&hzD)==9);
  }
  CHECK3(t10.g==10 && t10.d==-10 && hzG==200.0f && hzD==-100.0f);
  Serial.sortie.clear();
  envoyer("etat");
  CHECK3(Serial.sortie.find("[ROUES] SC gauche D2/GPIO25 : fronts=100 rejets=1 200.0 imp/s ; "
                            "droite D3/GPIO7 : fronts=50 rejets=0 -100.0 imp/s\n")!=std::string::npos);
  envoyer("p 0 0 0");
  jusqua(horloge+700);                          // plus de front : frequences nulles
  CHECK3(derniereT(t) && champsDerniereT()==10 && Serial.sortie.rfind(" 0.0 0.0\n")==Serial.sortie.size()-9);
  Serial.sortie.clear();
  envoyer("roues 0");
  CHECK3(Serial.sortie.rfind("[ROUES] Lecture SC coupee ; D2/D3 en entree haute impedance.\n",0)==0);
  CHECK3(!interruptions[D2] && !interruptions[D3] && modes[D2]==INPUT && modes[D3]==INPUT);
  jusqua(horloge+60);
  CHECK3(champsDerniereT()==8);
  Serial.sortie.clear();
  envoyer("etat");
  CHECK3(Serial.sortie.find("[ROUES]")==std::string::npos);
  envoyer("tel 0");
  Serial.sortie.clear();
  envoyer("roues 2");                          // l'etat periodique peut preceder la reponse
  CHECK3(Serial.sortie.find("[ERREUR] Commande invalide.")!=std::string::npos && !interruptions[D2]);

  // Centrale (programme principal seulement).
  Serial.sortie.clear();
  envoyer("imu 50");
#ifdef FIELDMIND_ROVER_COMPLET
  CHECK3(Serial.sortie=="[CENTRALE] Lignes I a 50 Hz ; ligne IMU texte suspendue.\n");
  CHECK3(actionneurs::cadenceCentrale()==50);
  envoyer("etat");
  CHECK3(Serial.sortie.find("imu=50 Hz")!=std::string::npos);
  CHECK3(!actionneurs::remiseLacetDemandee());
  envoyer("lacet0");
  CHECK3(actionneurs::remiseLacetDemandee() && !actionneurs::remiseLacetDemandee());
  envoyer("wd 300");
  envoyer("tel 10");
  // Perte USB : retour aux valeurs du demarrage (chien coupe, tel 0, imu 0).
  Serial.connectee=false; jusqua(horloge+260);
  CHECK3(actionneurs::cadenceCentrale()==0);
  Serial.connectee=true; Serial.sortie.clear(); actionneurs::actualiser();
  CHECK3(Serial.sortie.find("[PILOTAGE] Chien=0 ms (coupe) ; inv=200 ms ; tel=0 Hz ; imu=0 Hz")!=std::string::npos);
#else
  CHECK3(Serial.sortie=="[CENTRALE] Absente de ce programme ; commande ignoree.\n");
  CHECK3(actionneurs::cadenceCentrale()==0);
#endif

  // Defaut verrouille : p refuse, reglages toujours acceptes, T signale le defaut.
  envoyer("g 5"); envoyer("d 7");
  jusqua(horloge+101);
  sorties[D9].active=false;
  envoyer("d 8");
  CHECK3(!sorties[D0].active && !sorties[D9].active);
  Serial.sortie.clear();
  envoyer("p 5 5 0");
  CHECK3(Serial.sortie.find("[DEFAUT] Cause=")==0 && Serial.sortie.find("Sorties bloquees")!=std::string::npos);
  Serial.sortie.clear();
  envoyer("tel 50");
  CHECK3(Serial.sortie.find("[PILOTAGE] Telemetrie T a 50 Hz")==0);
  jusqua(horloge+25);
  CHECK3(derniereT(t) && t.defaut==1 && t.g==0 && t.d==0);
  std::printf("%d verifications controleur ROVER_3 OK (materiel simule).\n",nr);
  return 0;
}

int main(int argc, char **argv) {
  if (argc > 1 && std::string(argv[1]) == "rover3") return scenarioRover3();
  int n=0;
#define CHECK(c) do { ++n; if(!(c)) { std::fprintf(stderr,"ECHEC actionneurs %d\n",__LINE__); return 1; } } while(0)
  const bool migration = argc > 1 && std::string(argv[1]) == "migration";
  if (migration) {
    Preferences ancien;
    ancien.putBool("inv_g", false); // Ne pas ecraser la calibration gauche.
    ancien.putBool("inv_d", true); // Ancienne calibration droite ignoree.
    actionneurs::demarrer();
    CHECK(niveaux[D1]==0 && niveaux[D4]==0);
    CHECK(sorties[D0].duty==0 && sorties[D9].duty==0);
    Serial.entree="inversed 1\n"; actionneurs::actualiser();
    CHECK(niveaux[D1]==0 && niveaux[D4]==1);
    CHECK(ancien.getBool("inv_d_v2", false));
    // Redemarrage : retrouver la correction droite et conserver la gauche.
    actionneurs::demarrer();
    CHECK(niveaux[D1]==0 && niveaux[D4]==1);
    std::printf("%d verifications migration du sens droit OK.\n", n);
    return 0;
  }
  actionneurs::demarrer();
  CHECK(sorties[D7].active && sorties[D7].duty==1500);
  CHECK(sorties[D0].duty==0 && sorties[D9].duty==0);
  CHECK(niveaux[D1]==1 && niveaux[D4]==0);
  actionneurs::actualiser();
  CHECK(Serial.sortie.find(VERSION_ATTENDUE)!=std::string::npos);
  if(argc>1) {
    creationTache=false;
    CHECK(!actionneurs::demarrerTache());
    CHECK(!sorties[D0].active && !sorties[D9].active && !sorties[D7].active);
  } else {
    CHECK(actionneurs::demarrerTache());
    Serial.entree="m 10\n"; actionneurs::actualiser();
    horloge+=101; actionneurs::actualiser();
    CHECK(sorties[D0].duty==102 && sorties[D9].duty==102);
    // Changer un pourcentage n'affecte pas la consigne de l'autre roue.
    Serial.entree="g 7\n"; actionneurs::actualiser();
    CHECK(sorties[D0].duty==71 && sorties[D9].duty==102);
    Serial.entree="d 20\n"; actionneurs::actualiser();
    CHECK(sorties[D0].duty==71 && sorties[D9].duty==204);
    // Le gauche peut etre arrete/inverse sans interrompre le droit.
    Serial.entree="g 0\nsensg 1\n"; actionneurs::actualiser();
    CHECK(sorties[D0].duty==0 && sorties[D9].duty==204);
    CHECK(niveaux[D1]==0 && niveaux[D4]==0);
    // Une inversion a chaud est refusee et coupe tout, sans changer le DIR.
    Serial.entree="sensd 1\n"; actionneurs::actualiser();
    CHECK(sorties[D0].duty==0 && sorties[D9].duty==0 && niveaux[D4]==0);
    // Calibration droite : conserver le DIR gauche, persister la correction.
    Serial.entree="sens 0\ninversed 0\n"; actionneurs::actualiser();
    CHECK(niveaux[D1]==1 && niveaux[D4]==0);
    Preferences lecture;
    CHECK(!lecture.getBool("inv_d_v2",true));
    Serial.entree="inverseg 0\n"; actionneurs::actualiser();
    CHECK(niveaux[D1]==0 && niveaux[D4]==0);
    CHECK(!lecture.getBool("inv_g",true));
    // Erreur de sauvegarde : ne pas changer l'inversion appliquee.
    memoireEcriture=false;
    Serial.entree="inverseg 1\n"; actionneurs::actualiser();
    CHECK(niveaux[D1]==0 && !lecture.getBool("inv_g",true));
    memoireEcriture=true;
    Serial.entree="inverseg 1\ninversed 1\nm 10\n"; actionneurs::actualiser();
    horloge+=101; actionneurs::actualiser();
    // Calibration en roulant refusee, aucun changement de DIR/configuration.
    Serial.entree="inversed 0\n"; actionneurs::actualiser();
    CHECK(sorties[D0].duty==0 && sorties[D9].duty==0);
    CHECK(niveaux[D1]==1 && niveaux[D4]==1 && lecture.getBool("inv_d_v2",false));
    Serial.entree="m 10\n"; actionneurs::actualiser();
    horloge+=101; actionneurs::actualiser();
    Serial.entree="s -10\n"; actionneurs::actualiser();
    horloge+=1000; actionneurs::actualiser();
    CHECK(sorties[D7].duty<1500 && sorties[D0].duty==102);
    Serial.entree="m 0\nsens 1\n"; actionneurs::actualiser();
    CHECK(sorties[D0].duty==0 && sorties[D9].duty==0);
    CHECK(niveaux[D1]==0 && niveaux[D4]==0);
    Serial.entree="stop\n"; actionneurs::actualiser();
    CHECK(!sorties[D7].active);
    Serial.entree="centre\n"; actionneurs::actualiser();
    horloge+=1000; actionneurs::actualiser();
    CHECK(sorties[D7].active && sorties[D7].duty==1500);
    Serial.entree="m 5\n"; actionneurs::actualiser();
    horloge+=101; actionneurs::actualiser();
    CHECK(sorties[D0].duty>0);
    Serial.connectee=false; horloge+=251; actionneurs::actualiser();
    CHECK(sorties[D0].duty==0 && sorties[D9].duty==0 && !sorties[D7].active);
    Serial.connectee=true; actionneurs::actualiser();
    CHECK(sorties[D0].duty==0 && !sorties[D7].active);
    // Une faute d'ecriture d'un seul canal coupe les deux sorties.
    Serial.entree="g 5\nd 7\n"; actionneurs::actualiser();
    horloge+=101; actionneurs::actualiser();
    sorties[D9].active=false;
    Serial.entree="d 8\n"; actionneurs::actualiser();
    CHECK(!sorties[D0].active && !sorties[D9].active && !sorties[D7].active);
    CHECK(niveaux[D0]==0 && niveaux[D9]==0);
  }
  std::printf("%d verifications controleur ROVER_2 OK (materiel simule).\n",n);
}
