# Préparation locale de la mission drone

Le drone peut être éteint et le Pi débranché : l'atelier tourne sur ce Mac. Aucun
vol ni déploiement réel n'a été effectué pour cette préparation. Le branchement au
CooleerX n'est pas déclaré validé par un résultat de simulation.

## Essayer maintenant

Depuis le projet :

```sh
software/rover/deploiement/lancer_atelier_drone.sh
```

Ouvrir <http://127.0.0.1:8096>, **Pilotage → Atelier drone — simulation locale complète**.
Le lanceur sélectionne le matériel simulé, une écoute sur localhost, des réglages
séparés dans `.donnees-atelier/`, et désactive les modèles IA. Il ne contacte pas le
Pi, n'ouvre pas de port série réel et ne démarre pas la découverte BLE physique.

- **Simuler la mission complète** : monte à 1 m, prend six photos autour d'un tour
  mesuré, attend la caméra du bas puis se recentre sur la croix et descend par paliers
  de 15 cm jusqu'à 22 cm au-dessus du plateau, avant le posé final.
- **Essayer le pilotage manuel**, puis **Décoller en simulation** : monte vers 70 cm.
  Maintenir **Monter/Descendre** change la consigne de hauteur ; relâcher conserve
  cette hauteur cible. Les autres boutons commandent lacet, translation avant/arrière
  et gauche/droite. **Caméra du bas simulée** change le capteur du banc.
- **Interrompre et poser en simulation** déclenche le retour au sol. La perte du
  battement de la manette pendant 0,5 s provoque aussi ce repli ; aucune reprise
  automatique de mouvement à la reconnexion ou au retour du focus.
- Choisir un scénario de panne pour vérifier le comportement. Les archives du
  dernier essai sont accessibles par **Photos simulées / Rapport de simulation**.
  Les fichiers sont gardés dans `~/.local/share/fieldmind/drone-simulations/`.

Les commandes de cet atelier utilisent `drone_atelier` et des images
`image.drone_simule`, distinctes du pilote et du flux du vrai drone. Le serveur
ne possède aucun bouton de lancement automatique au redémarrage.

## Boucle de commande préparée

`fieldmind/drone/autopilote.py` produit des **vitesses bornées**, pas des positions
inventées ni des paquets propres à un matériel. Il reçoit hauteur, cap, état au sol,
identité de caméra et cible visuelle datés. Le banc simule les capteurs hauteur/cap ;
ces capteurs ne sont pas disponibles par magie sur le CooleerX.

La rotation est suivie en déroulant le cap autour de ±180°. Chaque secteur est
photographié après stabilisation, puis le tour est achevé avant le changement de
caméra. Une caméra demandée n'est pas considérée active avant une image récente
identifiée comme provenant du capteur du bas.

La descente suit la boucle : **détecter → recentrer → stabiliser → descendre d'un
palier → vérifier à nouveau**. Si la croix sort de la zone de centrage, le palier
est suspendu. Si la cible disparaît, la vitesse verticale demandée revient à zéro ;
une perte durable déclenche un atterrissage de repli, uniquement après envol confirmé.
Une commande d'atterrissage ne sert jamais à prouver que le posé a réussi : le retour
au sol doit être observé. Après perte de toute estimation, le banc signale « posé non
confirmé » même si la dynamique simulée a effectivement touché le sol.

La stabilisation d'attitude devra rester dans le contrôleur embarqué du drone. Le
Pi tient la mission et le battement des consignes ; il ne remplace pas une centrale
inertielle par cinq images par seconde. Les gains vitesse/axes RC et les signes
réels devront être mesurés avant de relier ces sorties à un aéronef.

## Pourquoi le bouton Décoller pouvait ne rien faire

Le pilote initial envoyait une trame générique isolée sur la caractéristique BLE
qui reçoit aussi des commandes vidéo. Aucun test ne prouvait que le contrôleur
moteur comprenait cette trame. « Il faut monter les gaz après le clic » est une
hypothèse, pas une procédure établie pour cet exemplaire.

Sur la variante **Wi-Fi RCFPVPro** documentée par
[e88pilot](https://github.com/Volonid/e88pilot/blob/master/backend/protocols/e88pro_rc_protocol_adapter.py),
il faut une session ouverte depuis UDP local 7070, puis un flux RC continu depuis
7060 vers `192.168.28.1:7080`, environ toutes les 40 ms. Son format diffère du simple
`66 … 99`. Le même bit y bascule décollage/atterrissage : le répéter peut être incorrect.
Cette variante annonce l'absence de télémétrie et une commande de caméra non confirmée.

`fieldmind/drone/protocoles.py` prépare cet encodage, les checksums, le battement à
25 Hz, l'expiration des consignes et des actions impulsionnelles. Il n'est **pas**
branché sur le BLE CooleerX. Une ouverture de transport UDP exige explicitement une
fiche de validation liée à l'identité, au protocole et au SHA-256 d'une capture,
avec essais des axes, de l'arrêt et de la perte de liaison. Aucune fiche de validation
réelle n'est fournie. Aucune connexion matérielle ne s'ouvre lors d'un import.

Le battement ne déduit pas « en vol » de l'envoi de takeoff : un retour observé doit
appeler `confirmer_en_vol(True)` avant un atterrissage automatique. Il ne réarme
jamais une session après arrêt ou expiration. L'interface matérielle précédente
reste bloquée jusqu'à identification du protocole de **cet exemplaire**.

## Répartition Pi / ASUS Ascent GX10

- **Pi** : réception, horodatage des images, mission, commandes bornées et battement
  RC indépendant. La vision de croix locale reste possible (mesure précédente sur Pi :
  environ 10–16 ms/image à 240 × 320).
- **GX10 facultatif** : détection de la croix et estimation relative à partir des images.
  Le service préparé n'a aucun endpoint de vol et n'utilise aucun LLM. Il retourne
  uniquement une cible, la séquence d'image et l'identifiant de calibration.
- **Client Pi** : une requête en vol à la fois, délai maximum de 300 ms ; âge évalué
  avec l'heure locale de capture, jamais avec l'heure de réception ou celle du GX10.
  Mauvaise séquence, mauvaise calibration, NaN ou résultat vieux de plus de 500 ms
  sont ignorés. Le retard du GX10 suspend la descente et finit par provoquer le repli.

L'algorithme métrique utilise une croix `+` plane de taille connue, la calibration
optique et une caméra approximativement perpendiculaire au plateau clair. Il estime
la hauteur par l'aire de la croix, ce qui évite de prendre sa rotation pour un changement
d'échelle. Les hypothèses d'inclinaison et de sol plan doivent être vérifiées sur le
vrai drone ; ce n'est pas une estimation générale de pose 3D ou une localisation GPS.
Une croix sans calibration donne seulement un écart dans l'image.

Exemple de service local avec la calibration du **simulateur seulement** :

```sh
cd software/rover
# Définir FIELDMIND_DRONE_VISION_JETON dans l'environnement, sans le publier.
.venv/bin/python -m fieldmind.drone.vision_service \
  --calibration config/drone_vision_simulation.json --hote 127.0.0.1 --port 8092
```

Sur le GX10, installer Python, NumPy, OpenCV et aiohttp et fournir une calibration
réelle avant emploi matériel. Aucun service GX10 n'a été installé ni contacté ici.
Le jeton est obligatoire, le serveur limite la taille/dimension des JPEG et refuse
d'accumuler des images lorsqu'il est occupé. Le client est dans `vision_distante.py`.

## Essais à reprendre au retour du drone chargé

1. Retrouver le protocole de l'application réellement utilisée et relever les
   échanges à l'arrêt, puis pour caméra, calibration, armement et sticks. Pour une
   application Android, un journal HCI exporté du **téléphone** est utile pour le BLE,
   et une capture réseau de l'application pour le Wi-Fi. `btmon` sur le Pi ne permet
   pas à lui seul de capturer une liaison établie entre le téléphone et le drone.
2. Banc hélices retirées : valider l'identité et les trames du transport, la cadence,
   le neutre, les signes, la calibration, l'armement/décollage et les arrêts. Ne pas
   tester des séquences d'armement au hasard en intérieur hélices montées.
3. Confirmer le basculement de caméra, son orientation et sa calibration. Sans image
   ventrale identifiée, le moteur de mission n'autorise pas l'approche sur la croix.
4. Établir une estimation de hauteur et de cap pendant toute la mission. L'aire de
   la croix fournit une hauteur relative quand elle est visible ; elle ne donne pas
   une hauteur pendant le scan avec la caméra avant. Il faudra un retour de vol ou
   une autre estimation validée pour cette phase.
5. Régler les axes et les gains, puis essais supervisés séparés : montée/maintien,
   lacet, translations, visée, paliers sur grande cible au sol, puis remorque immobile.

Ces essais physiques restent nécessaires : un logiciel de simulation ni un GX10 plus
puissant ne peuvent établir à distance comment réagit un drone éteint.

## Réexécuter la validation

```sh
cd software/rover
.venv/bin/python -m fieldmind.drone --scenario tous --graines 20 --sortie ../../tmp/drone-validation
.venv/bin/python -m pytest -q -m 'not lent'
.venv/bin/ruff check fieldmind tests
node --test tests/web/*.test.mjs
```

Le banc fait varier les positions initiales, ajoute une inertie et une dérive et
rend des images synthétiques analysées par le détecteur. La cible n'est pas injectée depuis
la position réelle du simulateur. Sa dynamique reste simplifiée : les précisions
obtenues dans ces essais ne sont pas des précisions promises sur le drone réel.

## Résultats de cette préparation locale

- **420 tests Python réussis**, deux scénarios longs existants exclus ; **22 tests
  JavaScript réussis**, lint et format des nouveaux modules vérifiés.
- **140 simulations** : vingt graines par scénario. Quarante missions `normal/derive`
  terminées avec six photos et au moins cinq paliers. Les cent autres essais
  produisent le repli ou l'état de posé non confirmé attendus, sans réarmement.
  [Synthèse des essais](validation/drone/synthese-2026-10-20.json).
- Contrôle par le navigateur local : lancement de mission, six photos, tour mesuré,
  bascule du capteur, six paliers et contact simulé ; puis essai manuel de décollage,
  montée, maintien d'altitude, lacet, descente, caméra du bas et interruption/posé.
- Prévisualisation locale relancée avec le lanceur isolé, IA désactivée. Aucun
  accès au Pi après la consigne de travail hors ligne, aucun déploiement ni vol réel.

L'absence de validation des moteurs sur ce drone reste un travail physique à reprendre,
avec les étapes ci-dessus. Les scénarios de repli testent une logique logicielle :
ils ne prouvent pas que la trame de repli est comprise par le CooleerX.
