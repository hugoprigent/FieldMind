# État et validation — 23 octobre 2026

| Élément | Preuve disponible | Limite |
| --- | --- | --- |
| Rover physique | Le rover roule ; navigation autonome testée en intérieur le 21 octobre avec le GX10 (« explore la pièce », trajet aller-retour sur la carte lidar) | Pas de mission extérieure validée ; pas de mesure de taux de réussite |
| Logiciel du rover | 756 tests Python réussis dans un venv neuf (Python 3.13), 2 scénarios longs désélectionnés ; 22 tests web | Pilotes et modèles remplacés par des simulateurs de test ; pas une mesure GX10 |
| Inférence GX10 | Huit modèles servis sur le GB10 ; analyses de scène d'environ 6 à 9 s pendant les essais réels | Pas de benchmark de mission ; GX10 distant joint par réseau privé |
| Remorque | Fabriquée ; volets commandés par servos (firmware ROVER_4 flashé) | Tractage autonome non validé ; pas de retour de position des volets |
| Drone | Flux vidéo reçu sur le Pi, batterie reçue ; mission complète en simulation | Aucun vol autonome validé ; niveau de charge non mesuré de façon fiable |
| CAO | Pièces, plans et contrôles géométriques du projet | Certains composants restent approximés ; coque non montée ; support SG90 non validé sous charge |

Détails : [état au 23 octobre](status-2026-10-23.md).

## Reproduire les tests du code courant

Après l'installation du [README](../README.md), depuis `software/rover` :

```bash
.venv/bin/python -m pytest -q -m "not lent"
node --test tests/web/*.test.mjs
```

Les tests de fonctions IA utilisent des faux serveurs sauf vérification explicite via `deploiement/gx10/verifier.py --profil gx10`. Le vérificateur contacte de vrais services ; aucun résultat de faux serveur ne doit être présenté comme une mesure d'inférence locale.

La configuration des modèles complémentaires sur le GX10 est en intégration. Les paramètres de la CAO et du code sont des valeurs initiales, à calibrer sur le montage réel. La présence d'une fonction dans le logiciel ne signifie pas qu'elle a été validée sur le terrain.

## Vérification de l'édition du 23 octobre

Installation neuve dans un venv Python 3.13 (uv) depuis `software/rover/requirements-ci.txt` :
**756 tests rover** réussis (`-m "not lent"`, 2 désélectionnés), **22 tests web rover**, Ruff 0.16.9
sans erreur. Le firmware ESP32-C5 compile (`pio run -e esp_c5`). Le serveur de la simulation
publique (`python -m fieldmind.public_demo`, profil `aucun`) répond en HTTP 200 sur `/`,
`/js/main.js` et `/api/sante`. Les liens Markdown locaux pointent vers des fichiers publiés.
La première démo MuJoCo n'a pas changé depuis l'édition du 21 octobre et n'a pas été relancée
localement ; elle reste testée par le job `demo` de GitHub Actions. Résumé :
[`checks-2026-10-23.json`](checks-2026-10-23.json). Ces contrôles sont locaux au Mac, pas des
benchmarks GX10.

## Vérification de l'édition du 21 octobre

Une installation neuve dans la copie de publication a également passé **425 tests rover**, **22 tests web rover**, **98 tests de la première démo** et **3 tests web démo**. Ruff passe sur le rover ; son simulateur sert interface, état et image caméra en HTTP 200. Les commandes et limites sont dans [`checks-2026-10-21.json`](checks-2026-10-21.json). Ces contrôles sont locaux au Mac, pas des benchmarks GX10.

Le firmware **ESP32-C5** compile également dans cette copie avec PlatformIO 6.2.0 (`pio run -e esp_c5`), sans téléversement sur le matériel.

## Vérification sur GitHub Actions

Les trois jobs **`rover`**, **`demo`** et **`firmware`** du
[run GitHub Actions 39510932511](https://github.com/hugoprigent/FieldMind/actions/runs/39510932511)
ont terminé avec succès pour le
[commit `c1562b3`](https://github.com/hugoprigent/FieldMind/commit/c1562b35b958a626d3878688a7d8265450dd447f).
Ce run vérifie le code et la compilation sous Linux ; il ne constitue pas un essai du
rover physique. Les finitions documentaires ultérieures ne changent pas le code testé.
