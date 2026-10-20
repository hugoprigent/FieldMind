# Conception mécanique

Le prototype courant utilise une base en contreplaqué de 10 mm, une direction par servo central, des roues avant imprimées à jantes rigides et pneus TPU à crampons, et les motoréducteurs arrière récupérés d'un robot tondeuse.

- [`prototype_final/`](prototype_final/README.md) : coque, supports, renforts, plans, visserie et pièces courantes. Les pièces imprimables sont dans [`exports/stl/`](prototype_final/exports/stl/).
- [`direction V04`](mounts/prototype/direction_v04_servo_traversant/README.md) : principe de direction, roues et historique de ses réglages. Le prototype final apporte des modifications ultérieures, dont la planche de 10 mm et les renforts ; utiliser ses pièces mises à jour en priorité.
- [`remorque`](prototype_final/remorque/README.md) : conception de la remorque, encore en préparation physique.

## Sources Fusion

Les scripts requièrent Autodesk Fusion et un document ouvert correspondant au modèle qu'ils modifient. Définir la variable d'environnement `FIELDMIND_ROOT` vers la racine locale de ce dépôt, ou exécuter depuis cette racine. Ce sont des scripts de construction et de modification du modèle de travail ; ils ne remplacent pas un assemblage complet autonome et peuvent modifier le document ouvert.

Les STEP externes et les assemblages les incorporant ne sont pas distribués dans cette édition, afin de ne pas leur attribuer une licence non vérifiée. Pour reconstruire ces parties, les références sont :

- Arduino Modulino Knob : [documentation et CAO officielles](https://docs.arduino.cc/hardware/modulino-knob/).
- SG90 : [bibliothèque FreeCAD, Servos/SG-90](https://github.com/FreeCAD/FreeCAD-library/tree/master/Electrical%20Parts/Servos/SG-90).
- RPLIDAR C1 : géométrie constructeur à obtenir auprès de SLAMTEC, selon ses conditions.

Les STL de pièces FieldMind sont disponibles directement. Vérifier le montage et les cotes du matériel réel avant fabrication ; une validation géométrique n'est pas une validation mécanique en charge.
