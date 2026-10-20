# État et validation — 21 octobre 2026

| Élément | Preuve disponible | Limite |
| --- | --- | --- |
| Rover physique | Pilotage manuel fonctionnel confirmé par Hugo | Pas encore de mission autonome physique validée dans cette édition |
| Logiciel du rover | 425 tests Python réussis sur Mac, 2 scénarios longs désélectionnés, dans l'arbre de développement le 21 octobre | Nombreux pilotes et modèles remplacés par des simulateurs de test ; pas une mesure GX10 |
| Inférence GX10 | Trois appels HTTP réels, avec réponses non vides du modèle `nvidia/Qwen3.6-35B-A3B-NVFP4`, sur le GB10 | Requêtes textuelles très courtes ; ni benchmark de navigation, ni évaluation de qualité de mission |
| Drone | Atelier de mission et tests de repli simulés | Aucun vol physique autonome validé |
| CAO | Pièces, plans et contrôles géométriques du projet | Certains composants restent approximés ou à mesurer ; coque non montée, remorque non disponible au moment de la préparation |

## Reproduire les tests du code courant

Après l'installation du [README](../README.md), depuis `software/rover` :

```bash
.venv/bin/python -m pytest -q -m "not lent"
node --test tests/web/*.test.mjs
```

Les tests de fonctions IA utilisent des faux serveurs sauf vérification explicite via `deploiement/gx10/verifier.py --profil gx10`. Le vérificateur contacte de vrais services ; aucun résultat de faux serveur ne doit être présenté comme une mesure d'inférence locale.

La configuration des modèles complémentaires sur le GX10 est en intégration. Les paramètres de la CAO et du code sont des valeurs initiales, à calibrer sur le montage réel. La présence d'une fonction dans le logiciel ne signifie pas qu'elle a été validée sur le terrain.

## Vérification de cette édition publique

Une installation neuve dans la copie de publication a également passé **425 tests rover**, **22 tests web rover**, **98 tests de la première démo** et **3 tests web démo**. Ruff passe sur le rover ; son simulateur sert interface, état et image caméra en HTTP 200. Les commandes et limites sont dans [`checks-2026-10-21.json`](checks-2026-10-21.json). Ces contrôles sont locaux au Mac, pas des benchmarks GX10.

Le firmware **ESP32-C5** compile également dans cette copie avec PlatformIO 6.2.0 (`pio run -e esp_c5`), sans téléversement sur le matériel.
