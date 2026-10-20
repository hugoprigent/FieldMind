# Crédits et composants tiers

## Code web embarqué

- **three.js** : code distribué dans `software/interface/web/vendor/three/` et `software/rover/fieldmind/web/vendor/three/`, sous licence MIT. Chaque copie conserve son fichier `LICENSE`. Les bibliothèques auxiliaires, dont le décodeur meshoptimizer, conservent leurs mentions dans les fichiers sources.
- Les autres bibliothèques Python, Arduino, ROS et JavaScript sont installées par leurs gestionnaires. Elles ne sont pas incorporées dans le code original de FieldMind. Les versions demandées sont indiquées dans les fichiers de dépendances et PlatformIO.

## Simulation et cartographie

- Les mondes ROS 2 référencent des ressources Gazebo Fuel d'**Open Robotics**, téléchargées à l'exécution. Les crédits et licences CC0 / CC BY 4.0 sont détaillés dans [`CREDITS.md`](software/ros2/src/fieldmind_simulation/CREDITS.md).
- Le poste peut afficher des cartes **OpenStreetMap**, avec l'attribution présentée dans l'interface. Un accès à Internet est requis pour ces fonds de carte.
- Le modèle `software/interface/web/modeles/rover_v3.glb` est l'ancien concept FieldMind, exporté pour la démo et distinct du prototype physique courant.

## Conception électronique

Les schémas et empreintes emploient des symboles des bibliothèques officielles **KiCad** ainsi que des blocs et modèles simplifiés créés pour ce prototype. La [licence des bibliothèques KiCad](https://www.kicad.org/libraries/license/) s'applique aux éléments issus de ces bibliothèques ; aucun changement de licence n'est prétendu ici.

## CAO et références

Les pièces imprimables et leurs scripts de construction sont ceux conçus pour FieldMind. Des études anciennes ont consulté **OpenRC Truggy, Daniel Norée, CC BY 3.0** ; les archives de référence téléchargées ne sont pas reprises dans cette édition.

Les STEP de composants Arduino Modulino Knob, bibliothèque FreeCAD SG90, lidar constructeur et supports tiers utilisés dans des études ne sont pas redistribués dans cette copie. Les assemblages complets qui peuvent incorporer ces géométries sont également exclus tant que leur redistribution n'a pas été vérifiée. Les sources à retrouver sont indiquées dans [`cao/README.md`](cao/README.md).

## Modèles d'IA

Les fichiers de configuration proposent des modèles Qwen / NVIDIA, RoboBrain, D-FINE, Depth Anything et SAM. Leurs poids ne sont pas inclus. Chaque téléchargement reste soumis aux conditions de son éditeur ; la présence d'un identifiant dans la configuration n'affirme ni une licence commune ni une validation sur le rover.
