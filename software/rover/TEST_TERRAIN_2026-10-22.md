# Mission du 22 octobre : retrouver un colis et revenir à la base

But : le rover mémorise le départ, cherche une caisse rouge au milieu de quelques
arbustes, rejoint un point libre devant elle, prend une photo et revient au départ.
Cela vérifie perception continue, mémoire spatiale, exploration progressive,
évitement et enchaînement d'actions. Le drone pourra ensuite fournir un premier
repérage aérien de la même mission.

## Installation simple

- Choisir une pelouse assez plate et tondue, avec des passages plus larges que le rover.
- Poser une vraie caisse rouge, assez grande pour être reconnaissable (environ
  40–60 cm), à 4–6 m du départ. La laisser partiellement visible depuis au moins
  un passage, avec quelques arbustes autour. Éviter un cache complètement opaque
  pour ce premier essai.
- Garder le poste et le GX10 connectés. Vérifier deux observations successives
  dans le journal et une carte qui se construit avant de lancer la recherche.
- Garder l'arrêt d'urgence accessible. Une branche rigide, un tronc ou un relief
  restent des obstacles ; le filtrage d'herbe ne les rend pas franchissables.

## Consignes à donner

1. Au départ, demander **« Mémorise ici le lieu base »**. Vérifier que « base »
   apparaît dans les lieux. Cette étape est séparée afin que le retour cible la
   position réellement mémorisée, même si le démarrage a eu lieu ailleurs.
2. Passer en mode auto et demander **« Trouve la caisse rouge puis prends une
   photo puis reviens à la base »**. La recherche inclut l'approche de la cible.
3. Suivre les points de vue, les objets et le trajet dans la carte. Un objet suivi
   de « ? » est encore une hypothèse ; deux analyses visuelles sur des captures
   distinctes sont exigées avant l'approche. Le détecteur rapide seul ne confirme
   pas un repère sémantique ; ses mesures restent utilisées pour les obstacles.

Le retour vers un lieu mémorisé dans la session utilise son odométrie, sans
déclencher une marche d'alignement GPS. Le nom « base » ne doit pas être remplacé
par une coordonnée générée par le modèle. « Retour au départ » désigne l'origine
de la session et peut différer du lieu base.

## Réussite vérifiable

- La cible est reconnue plusieurs fois et reliée à une position mesurée.
- Les points d'exploration et les trajets de recherche restent dans l'espace connu accessible.
- Le rover s'arrête devant la caisse sans la toucher ; une photo est réellement
  enregistrée dans `~/fieldmind/photos` sur le Raspberry.
- Il revient près de la base mémorisée, avec son trajet et les observations visibles.

En cas d'objet absent, de passage inconnu ou bloqué, de perte caméra/GX10 ou de
confirmation insuffisante, la mission doit s'arrêter avec une raison. L'absence
de cible est un résultat de recherche, pas un emplacement à inventer.

## Progression des essais

1. Caisse visible à 2–3 m, sans arbuste entre le rover et elle.
2. Même mission à 4–6 m avec un arbuste qui impose un autre point de vue.
3. Ajouter une caisse d'une autre couleur : seule la caisse rouge doit compter.
4. Retirer la caisse rouge : vérifier l'échec explicite sans faux succès ni point inventé.

La vue 3D représente des mesures locales et des estimations de volume. Ce n'est
pas une reconstruction métrique complète d'un jardin ou d'une maison ; les petits
échos isolés sont filtrés dans l'affichage, tandis que les mesures brutes restent
disponibles à la navigation et à la sécurité.
