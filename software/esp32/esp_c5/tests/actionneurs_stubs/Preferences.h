#pragma once
#include <map>
#include <string>
#include <cstddef>
extern bool memoireEcriture;
class Preferences {
  inline static std::map<std::string,bool> donnees;
public:
  bool begin(const char *, bool) { return true; }
  bool getBool(const char *cle, bool defaut) { return donnees.count(cle) ? donnees[cle] : defaut; }
  size_t putBool(const char *cle, bool valeur) {
    if(!memoireEcriture) return 0;
    donnees[cle]=valeur; return sizeof(bool);
  }
};
