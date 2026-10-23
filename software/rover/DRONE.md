**Français** · [English](DRONE.en.md)

# Drone : reconnaissance et statut du pilotage

État public au 23 octobre 2026. La mission complète existe dans le [simulateur](DRONE_MISSION.md),
mais aucun vol réel n'est validé.

## Ce qui a été observé

- Le Raspberry Pi reçoit des images d'un petit drone à double caméra par BLE. Le flux vidéo et la
  capture d'images ont été vérifiés sur le matériel ; les images mesurent 240 × 320 pixels.
- Le logiciel peut enregistrer six vues, produire un montage et un rapport, puis détecter une croix
  à l'image pour évaluer son centrage. Cette reconnaissance se fait sans commander les moteurs.
- Le dépôt contient un encodeur expérimental de trames, établi par analyse hors ligne de la version
  1.1.0 de l'application d'origine, et des tests de vecteurs simulés. Cela ne prouve ni que le drone
  accepte les trames, ni que les axes, le battement ou l'arrêt fonctionnent. Le transport RC n'est
  pas raccordé et aucune commande de vol n'est activée dans le pilote.
- Le niveau de batterie n'est pas connu de manière fiable. Les mesures d'IMU du rover ne donnent pas
  l'altitude ni le cap du drone.

Les adresses radio, l'application tierce, ses ressources et les notes d'accès ne sont pas publiées
dans ce dépôt. Le pilote réel reste bloqué jusqu'à des essais au banc reproductibles.

## Reconnaissance sans vol

Dans **Pilotage > Flux Drone** :

1. Vérifier que l'image reçue est récente.
2. Tourner le drone manuellement entre les vues, puis choisir **Capturer vue** six fois. Ce test
   n'envoie aucune commande aux moteurs et ne bascule pas lui-même la caméra.
3. Consulter le montage et le rapport enregistrés localement sur le Pi.
4. Utiliser **Viser la croix** pour vérifier la détection et le centrage à l'image. Une croix centrée
   n'atteste pas un atterrissage.

La détection géométrique utilise OpenCV, sans LLM ni clé API. Une perte prolongée du flux arrête
l'analyse ; elle ne reprend pas automatiquement à la reconnexion. La bascule entre caméras reste
expérimentale et doit être vérifiée visuellement.

## Ce qui reste à valider

Avant toute commande réelle, il faut vérifier au banc, hélices retirées, le sens et la neutralité de
chaque axe, la cadence des trames, le comportement en cas de perte BLE et l'arrêt. Il faut aussi
identifier visuellement la caméra active, confirmer les données de vol disponibles et leurs unités,
puis tester un contrôleur avec un battement indépendant de la vidéo. Ensuite seulement, la mission
simulée pourra être adaptée à des essais supervisés au sol et en vol.

La [mission simulée](DRONE_MISSION.md) comprend décollage, tour de reconnaissance, recentrage et
retour au plateau. Ce scénario reste un outil de préparation ; ses résultats ne constituent pas une
preuve de vol.

## Vérifier le code sans matériel

Depuis `software/rover` :

```sh
.venv/bin/python -m pytest -q tests/test_drone.py
.venv/bin/ruff check fieldmind tests
node --check fieldmind/web/js/vues/pilotage.js
```
