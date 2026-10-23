**Français** · [English](README.en.md)

# FieldMind — direction V04 : tout sous la planche, servo traversant

Étude du 14 octobre 2026, document Fusion **`systeme de direction simple v1`**, groupe
`FM_Direction_V04` (la V03 (référence de travail non incluse) reste dans le document, masquée).

![Vue de dessous, braquage maximal](vues/02_dessous_iso_braquage_31deg.png)

## Principe

- **Planche horizontale** : l'arrière fixe la hauteur. Roues de **Ø200** (crampons compris),
  axe **30 mm sous la planche**, donc dessous de planche à **130 mm** du sol. Avec des roues
  avant Ø150, l'axe avant doit être **55 mm sous la planche** : c'est calculé automatiquement.
- **Par côté, deux pièces** :
  - un **support plat** fixé sous la planche par **2 vis M8** (tête prisonnière sous le support),
    avec un **écrou M8 prisonnier côté planche** pour la vis-pivot, et deux **butées** ;
  - une **fusée** imprimée à plat, avec deux **608ZZ** rapprochés (arête de 2 mm), le bras de
    direction intégré et, en bas, l'axe de roue (vis M8 dans un écrou prisonnier).
- **Toute la visserie tient en M8×30**, la plus longue vis disponible : la vis-pivot monte
  par-dessous (tête sous la fusée) et la vis de roue a sa tête logée dans le moyeu.
- **Servo DT996 à travers la planche de 5 mm** (découpe 41,4 × 21 mm), tête en bas. Les oreilles
  sont vissées sur le dessus de la planche ; le boîtier dépasse de 3,6 mm dessous (5,1 avec son
  bossage) et le **palonnier étoile d'origine** est 13,3 mm sous la planche.
- **Palonnier imprimé** : l'étoile s'encastre dans une empreinte de sa forme, creusée dans son
  plateau. Il est au niveau des bras et porte les deux biellettes. L'entraînement se fait par la
  forme ; **une vis M3 centrale** avec une rondelle large, vissée dans l'axe en laiton du servo,
  tient le palonnier imprimé et l'étoile.

![Palonnier imprimé : coupes dans le rebord, au fond de l'empreinte et au niveau des puits M2](vues/12_palonnier_etoile_coupes.png)
- **Biellettes** montées sur **3×10×4** des deux côtés. La vis M3 serre uniquement la bague
  intérieure (rondelle M3 de chaque côté), donc sans frottement.
- **Hauteurs** : sous la planche, la tringlerie descend à 28 mm et la fusée jusqu'à l'axe de
  roue + 13 mm. Le **point le plus bas est à 62 mm du sol**, juste sous l'axe de roue.

![Vue de face](vues/04_face_neutre.png)

## Paramètres Fusion

| Paramètre | Valeur | Effet |
| --- | ---: | --- |
| `d4_planche_larg` | **300 mm** | Largeur de planche. L'écartement des pivots la suit : `d4_E = d4_planche_larg + 2 × d4_debord_pivot` = **310 mm**, et les biellettes s'allongent (entraxe 140,8 mm). |
| `d4_debord_pivot` | 5 mm | Axe de pivot au-delà de chaque bord de planche. Le haut des pneus dépasse au-dessus du dessous de planche, donc **garder au moins 3 mm** : avec 5 mm, il reste 6 mm entre pneu et planche au braquage maximal. |
| `d4_planche_ep` | **5 mm** | Épaisseur de planche. Le servo posé dessus descend ou monte avec elle, et l'empreinte de l'étoile suit. Essai à 10 mm sans erreur. |
| `d4_etoile_sous_oreilles` | **18,3 mm** | Face des branches de l'étoile sous la face d'appui des oreilles (49 − 30,7, mesuré). Fixe la profondeur de l'empreinte et la hauteur du rebord du palonnier imprimé. |
| `d4_roue_ar_d` | 200 mm | Diamètre total des roues arrière. |
| `d4_axe_ar_sous_planche` | 30 mm | Axe arrière sous la planche. |
| `d4_roue_av_d` | 150 mm | Diamètre des roues avant. |
| `d4_axe_av_sous_planche` | dérivé : 55 mm | Axe avant sous la planche pour que la planche reste horizontale. Garder ≥ 55 mm, soit des roues avant ≤ Ø150 avec l'arrière actuel. |
| `d4_vis_roue`, `d4_ecrou_roue_s`, `d4_ecrou_roue_h` | 8 / 13 / 6,5 mm | Vis et écrou de roue (écrou M8 simple prisonnier). |
| `d4_planche_long`, `_avant` | **500** / 60 mm | Planche **30 × 50 cm** ; débord avant devant les pivots. |
| `d4_empattement` | 380 mm | Des pivots avant à l'axe des moteurs arrière (les supports arrivent à 6 mm du bord arrière). |

Essais réalisés dans Fusion :
- écartement de 220 et de 400 mm ;
- planche de 15 mm ;
- roues avant Ø130 (l'axe passe alors à 65 mm sous la planche) ;
- vis M10.

Tout se reconstruit, sans fonction ni liaison en erreur et sans collision
([mouvement_verifie.json](mouvement_verifie.json)).

Au-delà d'environ 340 mm d'écartement, la biellette dépasse 180 mm : il faut alors
l'imprimer en diagonale.

## Roues avant

![Roues, braquage maximal](vues/06_iso_roues_braquage_31deg.png)

**Diamètre conseillé : Ø150 total.** C'est le maximum possible avec l'arrière actuel (roues
Ø200, axe 30 mm sous la planche) : l'axe avant tombe alors 55 mm sous la planche. Un diamètre
plus petit fonctionne aussi, puisque la fusée s'allonge automatiquement. Ce n'est pas le
diamètre qui fait tenir la planche horizontale, mais la hauteur d'axe, qui est recalculée.

| Élément | Choix |
| --- | --- |
| Pneu | **TPU 90A** (le 70A ne passe pas à l'impression), épaisseur **15 mm crampons compris** (12 mm de gomme + 3 mm de crampons), 2 rangées décalées de 24 crampons de 7 mm. Crampons **trapézoïdaux biseautés à 45° aux deux bouts** : aucune face en surplomb au-delà de 45°, quel que soit le flanc posé sur le plateau (contrôlé sur le modèle). |
| Jante | PETG, portée du pneu **Ø120**, **6 rayons pleine largeur** de 4,5 mm raccordés par des congés (3 mm au moyeu, 5 mm au fût), rebord intérieur de 2 mm avec rampe à 45°. **Rebord extérieur = flasque vissé** (Ø124 × Ø96 × 3) : 6 vis M3×8 dans 6 inserts M3 à chaud logés au bout des rayons |
| Roulements | **2 × 608ZZ côte à côte côté fusée** dans le moyeu, séparés par une **arête de 2 mm de la jante** (Ø19,5) qui n'appuie que sur les bagues extérieures ; **entretoise imprimée de 2,2 mm** entre les bagues intérieures |
| Axe | Vis **M8×30** fixe. Sa tête est logée dans le moyeu (alésage Ø24 pour une douille de 13), elle traverse les deux 608ZZ et se visse dans l'**écrou M8 prisonnier** de la jambe de fusée (6 mm de filet en prise). Le roulement intérieur s'appuie sur la portée Ø12 de la fusée, qui ne touche que la bague intérieure. Rondelle imprimée Ø12 sous la tête. |

### La roue ne peut pas sortir de ses roulements

![Demi-coupe du moyeu](vues/19_coupe_moyeu_roue.png)

- **Ce que serre la vis d'axe** : tête → rondelle → bague intérieure du roulement extérieur →
  entretoise → bague intérieure du roulement intérieur → portée Ø12 de la fusée → écrou. Rien
  d'autre n'est serré. Les deux bagues intérieures sont donc fixes.
- **L'arête de la jante est prise entre les deux bagues extérieures**, comme dans un moyeu de vélo.
  - Si la roue tire vers l'extérieur, l'arête bute sur le roulement extérieur, retenu par la tête
    de vis.
  - Si la roue pousse vers la fusée, l'arête bute sur le roulement intérieur, retenu par la portée.
  - La jante ne peut pas sortir, même avec des roulements qui entreraient librement.
- **L'emmanchement (Ø22,15) ne tient rien axialement** : il empêche seulement les bagues
  extérieures de tourner dans le plastique. Un roulement qui entre à la main mais ne tombe pas
  suffit. S'il flotte, une goutte de colle cyanoacrylate ou de Loctite 641 sur la bague
  extérieure.
  - Ni insert ni vis de blocage : inutile.
  - Pas d'ajustement très serré non plus : il fendrait le moyeu et empêcherait le roulement
    extérieur de se placer au serrage.
- **Entretoise de 2,2 mm, pas 2,0**. Si l'entretoise est plus fine que l'arête, le serrage
  précontraint les deux roulements contre l'arête : la roue tourne dur. Avec 0,2 mm de plus, le
  roulement extérieur se cale tout seul en coulissant de 0,2 mm dans son logement, et la roue
  garde au plus 0,2 mm de jeu axial, sans effet.
  - Le dessous de l'arête s'imprime en surplomb. Son alésage est passé de Ø17 à Ø19,5 pour
    réduire ce surplomb de 2,6 à 1,3 mm : moins de bavure.
  - Ébavurer quand même au cutter avant de presser le roulement extérieur.
- **Jeux contrôlés dans Fusion** (`14_verifier_roues.py`, [verification_roues.json](verification_roues.json)),
  roues droites et braquées à ±31° :
  - jante → fusée : **1,5 mm** (le moyeu ne touche que par les roulements) ;
  - pneu → planche : **6 mm** au braquage maximal ;
  - flasque → toute pièce fixe : 35 mm.

### Pneu TPU 90A et flasque vissé

Un pneu en 90A est trop raide pour passer par-dessus un rebord fixe : il faudrait étirer son
diamètre intérieur de 119 à 124 mm (+4 %). Le rebord extérieur est donc devenu un **flasque
vissé** (`15_jante_flasque.py`).

![Roue montée, flasque vissé](vues/20_roue_flasque_visse.png)
![Jante sans pneu ni flasque : bossages des inserts M3](vues/21_jante_inserts_M3.png)

**Montage du pneu** :
- poser les 6 inserts M3 à chaud dans les bossages, par la face extérieure (trous Ø4,0 × 6,5,
  pour des inserts M3 de Ø4,6 × 5,7) ;
- flasque démonté, enfiler le pneu **à plat** sur la portée Ø120 par le côté extérieur :
  - la jante a un chanfrein d'entrée de 1 mm ;
  - les bords intérieurs du pneu sont chanfreinés de 2 mm ;
- le pousser jusqu'au rebord intérieur. Le serrage est de 1 mm au diamètre, donc il faut pousser
  fort. Quelques minutes dans l'eau à 60 °C assouplissent le TPU.
- poser le flasque et serrer les 6 vis M3×8 ;
- il reste 0,2 mm de jeu latéral de chaque côté, entre le pneu et les rebords.

**Réglages** :
- `d4_pneu_serrage` : à 1,5 mm si le pneu tourne sur la jante ; à 0,5 mm s'il est impossible à
  enfiler ;
- `d4_pneu_jeu` : le jeu latéral.

Autres paramètres : `d4_roue_av_d` (150), `d4_roue_larg` (35), `d4_pneu_ep` (15 maxi),
`d4_crampon_h` (3), `d4_crampon_n` (24), `d4_portee_roue_d` (12). Essais faits sans collision
jusqu'au braquage maximal, planche comprise.

Les roulements sont des 608ZZ (alésage 8), donc **les vis de roue et de pivot sont en M8** ;
les M10×40 ne servent pas ici.

### Impression des roues sur l'A1 mini

- **Jante (PETG)** : posée sur sa face extérieure (celle des trous d'inserts), sans support.
  - Les rayons sont des parois verticales : 3 parois et un remplissage gyroïde de 15 % suffisent,
    car les rayons de 4,5 mm sortent presque pleins. L'impression est rapide et la jante rigide.
  - Taille Ø124 × 33.
  - Les seuls surplombs sont deux marches en anneau au fond des logements de roulements (0,9 et
    1,3 mm). Elles s'impriment sans support.
- **Flasque (PETG)** : à plat, 2 parois, 3 couches pleines dessus et dessous. Taille Ø124 × 3.
- **Pneu (TPU 90A)** : à plat sur un flanc, **sans support**. Bobine en alimentation externe,
  hors AMS lite.
  - **Remplissage gyroïde 15–20 %, 3 parois**, 4–5 couches dessus et dessous. Le 90A est bien
    plus raide que le 70A : moins de remplissage garde un peu d'amorti.
    - Si le pneu est trop dur, descendre à 12 %.
    - S'il s'affaisse sous le rover, monter à 25 %.
  - Vitesse : 20–25 mm/s, première couche à 10 mm/s.
  - Taille Ø150 × 31, ce qui tient dans le plateau de 180 mm.
- **Imprimer d'abord une jante et un pneu** pour régler le serrage avant la paire.

### Tolérances dessinées (FDM, buse de 0,4 mm)

Les trous imprimés sortent en général 0,1 à 0,2 mm plus petits que le modèle ; les cotes
ci-dessous en tiennent compte.

| Endroit | Cote du modèle | Effet recherché |
| --- | --- | --- |
| Logements 608ZZ (Ø22) | Ø22,15 | Emmanchement léger : enfoncé à la main ou à l'étau, le roulement ne tombe pas |
| Logements 3×10×4 (Ø10) | Ø10,15 | Idem pour les biellettes |
| Arêtes entre roulements | Ø17 (fusées) ; Ø19,5 (jantes) | Appui sur les bagues extérieures seulement |
| Portées et rondelles imprimées | Ø12 | Appui sur les bagues intérieures seulement (pas de frottement des flasques) |
| Passages de vis M8 | Ø8,4 | Passage libre |
| Empreintes de tête M8 (supports) | 13,3 sur plats × 5,5 | La tête H M8 entre par-dessous, affleure et ne tourne pas |
| Passages de vis M3 (bras, palonnier) | Ø3,4 | Passage libre jusqu'à l'écrou prisonnier |
| Accès central du palonnier | Ø7 | Passage de la vis M3 centrale ; la rondelle large appuie autour |
| Passages Ø2,4 sous trois branches (palonnier) | Ø2,4 | Inutilisés : les trous de l'étoile n'acceptent que du M1 (M2 possible après perçage à Ø1,6) |
| Empreinte de l'étoile | contour mesuré + 0,25 mm | L'étoile entre à la main, sans jeu angulaire visible |
| Empreintes d'écrou M8 / M3 | 13,3 / 5,8 sur plats, +0,2 en profondeur | L'écrou entre à la main, ne tourne pas |
| Découpe du servo dans la planche | boîtier + 0,5 mm (41,4 × 21) | Pas de contact boîtier/bois ; le trou Ø26 du modèle (ancien palonnier rond) est inutile |
| Pièces qui tournent l'une contre l'autre | 1 mm (fusée/support, butées/tube) ; 0,5 mm (rondelle M3) | Aucun frottement |
| Pneu sur jante | Ø intérieur = Ø jante − 1 mm ; 0,2 mm de jeu de chaque côté | Le pneu tient sans colle ; flasque vissé pour le monter et le démonter |
| Inserts M3 à chaud (jantes) | Ø4,0 × 6,5 | Inserts M3 de Ø4,6 × 5,7 ; fer vers 230 °C pour le PETG |
| Entretoise de roue | 2,2 mm (arête : 2,0) | Pas de précontrainte des 608ZZ, 0,2 mm de jeu axial au plus |

Si les roulements entrent trop dur ou trop facilement sur ta machine, imprimer un petit anneau
d'essai et corriger le logement de ±0,05 mm. Les cotes sont dans `01_construire.py`
(`SIEGE_608`, `SIEGE_3104`) et `05_roues.py`.

![Jante à rayons (avant le flasque vissé)](vues/11_jante_rayons.png)

## Supports moteurs arrière

![Ensemble planche 30 × 50](vues/09_ensemble_planche_30x50.png)

Les supports moteurs sont dans leur propre document Fusion, « supports roues fieldmind », inséré
par référence dans le rover (`prototype_final/scripts/18_document_supports.py`).

**Fusées renforcées** (`prototype_final/scripts/19_fusees_renfort.py`) : la fusée est en L, et le coin
du L (pont de 8 mm entre le tube du pivot et la jambe) pliait. Un gousset triangulaire sur toute la
largeur (26 mm) le raidit, de 11 à 21 mm de l'axe du pivot et jusqu'à 44 mm sous la planche. La douille
de 13 passe toujours sur la tête de la vis-pivot, et la poche de l'écrou de roue reste libre. Les
fusées tournent avec la direction : elles restent dans le document du rover.

La pièce Onshape d'Hugo (sources/support_moteur_onshape.step (référence de travail non incluse))
tenait le moteur sur 37,5 mm avec 2 vis M8 en ligne : le moteur basculait. Elle est remplacée par
`prototype_final/scripts/16_supports_moteurs.py`, même interface moteur :
- **Moteur tenu sur 85 mm**, dans un alésage étagé comme lui (cotes de Hugo : réducteur Ø46 sur
  55,5 mm, puis moteur Ø42 sur les 58 derniers mm, 113,5 mm en tout) : Ø46,3 puis Ø42,3, soit 0,3 mm de jeu.
  Épaulement Ø52 de 2,5 mm et 4 trous Ø4 côté roue.
- **Collier** : fente de 2 mm sous le moteur, serrée par **2 vis M6×25** et leurs écrous dans une
  quille. Le moteur est serré quelle que soit la tolérance d'impression, et il ne tourne pas.
- **Semelle de 95 × 108 × 10 mm** sous la planche, **4 vis H M8** : les 2 trous d'origine
  (x ±130) et 2 nouveaux à x ±68 (y 335 et 425). Tête sous la semelle, rondelle et écrou sur la
  planche. M8×25 avec la planche de 5 mm, M8×30 avec 10 mm. Une encoche Ø14 laisse passer la
  vis M6 de la coque.
- **Placement** : face côté roue au ras du bord de planche, axe du moteur 30 mm sous la planche,
  à **380 mm** derrière les pivots. Même pièce des deux côtés, tournée de 180°.
- **Impression** : debout sur la face côté roue (alésage vertical), sans support ; 72 × 108 ×
  95 mm, environ 225 cm³ ; 4 parois, gyroïde 30 %.
- Point le plus bas : la quille, à 58 mm du sol.

**Les moteurs ne sont pas modélisés** : aucun modèle 3D de ces moteurs n'est disponible. Pour
les ajouter, il faut le STEP du fabricant, ou à défaut le diamètre et la longueur du corps,
l'épaulement, la longueur d'arbre et les entraxes de fixation.

![Dessous de l'ensemble](vues/10_ensemble_dessous.png)

## Capteurs : deux mâts en profilé 40 × 40

![Rover avec le mât des caméras à l'avant et le mât du lidar à l'arrière](vues/17_ensemble_capteurs.png)

Les deux capteurs sont montés chacun sur un profilé aluminium **40 × 40 type B** (rainure 10),
vertical et centré sur l'axe. Chaque profilé tient sur la planche par **deux équerres 40**, à gauche
et à droite, vissées en **M8**. Sur le haut du profilé s'emboîte une pièce imprimée, sur le même
principe pour les deux mâts :
- l'emboîtement fait 20 mm de haut, avec 0,25 mm de jeu par côté ;
- **4 languettes** entrent dans l'ouverture des rainures, donc la pièce ne tourne pas ;
- **2 vis M8 latérales** (gauche et droite, passages Ø8,6) se serrent dans des écrous en T M8 de la rainure.

Le support du lidar reprend ce principe avec un emboîtement de 31 mm et deux languettes seulement
(avant et arrière), voir plus bas.

Ces éléments sont construits par `11_lidar_stereo.py` dans le même groupe Fusion et contrôlés par
`12_verifier_capteurs.py` ([verification_capteurs.json](verification_capteurs.json)).

**Équerres** : géométrie d'une équerre 40 × 40 du commerce (fiche Emile Maurin 94-103, profil 40×40) :
- 38 mm de large, ailes de 36 mm ;
- trou Ø8,5 à **20 mm de la face du profilé**, ce qui place les trous de la planche à 40 mm de
  l'axe du profilé ;
- pour une autre équerre, poser l'équerre sur le gabarit et vérifier que son trou tombe sur la
  croix avant de percer.

### Mât lidar (RPLIDAR C1)

- **Place** : à l'aplomb de l'axe des roues arrière (`d4_lidar_y = d4_empattement`).
  - Le rover braque autour d'un point de cet axe. Le lidar y tourne sur lui-même sans balayer
    latéralement, ce qui déforme moins les scans.
  - C'est aussi l'origine habituelle (`base_link`) d'un robot à direction de type voiture.
- **Hauteur** : profilé de **90 mm** au-dessus de la planche (`d4_profil_h`), plus une **rehausse
  imprimée de 60 mm** (`d4_rehausse_lidar`) sous le support : les deux profilés disponibles font 90.
  - Le support est d'une seule pièce : une colonne octogonale de 56 mm, avec en bas un
    **emboîtement de 31 mm** enfilé sur le haut du profilé. Deux languettes vont dans les rainures
    avant et arrière. **2 vis M8×16 latérales** se serrent dans des écrous en T M8 des rainures
    gauche et droite, 23 mm sous le haut du profilé. Rien ne se visse par-dessous la planche.
  - Les têtes des vis restent sous le toit de la coque : on monte le mât complet avant de la poser.
    Elle passe par-dessus le lidar, avec une ouverture de 61 × 61 mm.
  - Les vis du C1 se serrent avec une clé Allen de 2, par les gorges des coins de la colonne.
  - Fait par `prototype_final/scripts/12_mats_profiles_90.py`, en place dans le document du rover.
  - Le plan laser est à **322 mm du sol**, comme le lidar simulé (0,323 m).
  - Il passe à 30 mm au-dessus des webcams.
  - Contrôle : 360 rayons dans le plan et à ±1,5° (planéité annoncée) ne touchent aucune pièce du
    rover.
- **Support imprimé** (`support_lidar_C1`) :
  - une plaque de 56 × 56 mm porte le C1, **aligné sur le rover** : son 0° (à l'opposé du câble,
    d'après la fiche Slamtec) regarde l'avant, et le câble part vers l'arrière dans une gorge ;
  - le C1 est vissé **par-dessous**, à travers 4 puits dans les coins de la jupe.
- **Le C1 se visse en M2,5, pas en M3**. La fiche Slamtec (archivée dans
  fusion_v01 (référence de travail non incluse)) indique 4 × M2,5 sur 43 × 43, avec
  **4 mm de vis au maximum dans le lidar**, sinon il est abîmé. Les passages du support sont Ø3,0.
  - Une M2,5×8 traverse 4,5 mm de plaque et entre de 3,5 mm dans le lidar.
  - Si une M3 entre vraiment à la main sans forcer, repercer les passages à Ø3,4 et garder M3×8.

![Mât lidar](vues/13_mat_lidar_C1.png)
![Support du lidar emboîté sur le profilé de 90 mm](../../../prototype_final/vues/11_mat_lidar_manchon.png)
![Support lidar vu de dessous : emboîtement, languettes, trous M8, gorges des vis du C1](../../../prototype_final/vues/13_support_lidar_dessous.png)

### Mât des caméras (deux webcams UGREEN 15728)

- **Profilé de 90 mm** au-dessus de la planche (`d4_cam_profil_h`), au bord avant : la barre et les
  webcams sont 5 mm plus bas que prévu (centres optiques à **265 mm** du sol au lieu de 270). Le champ
  des deux webcams reste dégagé ; la barre passe à 3,3 mm du nez de la coque.
  - Son axe est 25 mm derrière le bord (`d4_cam_y`), ses faces à 5 mm.
  - Sa longueur découle de la hauteur visée pour les objectifs ; le script l'arrondit au mm.
- **Barre imprimée** (`barre_stereo`, 170 × 56 mm) emboîtée sur le haut du profilé. Elle porte les
  deux caméras ; leur position relative ne dépend que de cette pièce, donc la calibration stéréo
  tient.
  - Le dessus de la barre est l'appui des caméras. Il est incliné de 10° vers l'avant, et
    l'emboîtement est vertical, donc incliné dans la barre.
  - **Le STL est déjà couché pour l'impression** : le dessus de la barre est à plat sur le plateau,
    ce qui donne une face d'appui bien plane.
  - Quatre goussets à 45° raidissent les bras. Ils restent à l'écart des vis M8 et des vis des
    caméras.
- **Géométrie** :

| Grandeur | Valeur | Paramètre | Pourquoi |
| --- | ---: | --- | --- |
| Écartement des centres optiques (base) | **120 mm** | `d4_stereo_base` | Le minimum physique est d'environ 85 mm (boîtiers de 81 mm). 120 mm donne une bonne précision à 4 m, distance où l'algo confirme les obstacles, et la barre tient sur le plateau de l'A1 mini. |
| Axes optiques | **parallèles** | – | Recouvrement maximal et rectification simple pour SGBM |
| Inclinaison vers le sol | **10°** | `d4_cam_inclinaison` | Même pose que la caméra simulée : moins de ciel, sol visible dès 0,42 m devant les objectifs |
| Hauteur des centres optiques | **270 mm** | `d4_cam_optique_sol` | Même hauteur que la caméra simulée (0,269 m), au-dessus de l'herbe courte et des roues avant (150 mm) |
| Position | objectifs au ras du bord avant | `d4_cam_y` | Rien du rover dans le champ, contrôlé par rayons sur 75° × 45° |

- **Ce que ça donne pour SGBM**. On compte en 640 × 360 avec un champ horizontal de 75°, soit une
  focale d'environ 417 px.
  - La précision vaut environ ±8 cm à 4 m et ±2 cm à 2 m, pour 0,25 px de disparité. C'est
    2,6 fois mieux que le module simulé (base de 60 mm).
  - Avec `numDisparities = 96`, la profondeur se mesure dès 0,52 m.
  - Avec 128, elle se mesure dès 0,39 m, donc jusqu'au bas de l'image. Le calcul coûte un tiers
    de plus.
  - Une base plus large tolérerait mieux le décalage entre les deux webcams, qui ne sont pas
    synchronisées. Mais la barre ne tiendrait plus sur le plateau, et le sol proche serait perdu.
- **Fixation des caméras** : une vis photo par caméra, par-dessous, à travers 8 mm de plastique.
  - Une webcam a presque toujours un filetage photo **1/4"-20**, pas M6. Essayer une vis M6 à la
    main, sans forcer. Si elle accroche, prendre des vis 1/4"-20 × 1/2" (12,7 mm).
  - Le trou de la barre (Ø6,8) laisse passer les deux.
- **Poses pour la TF ROS** : le repère `base_link` est au sol, sous le milieu de l'axe des roues
  arrière (x avant, y gauche, z haut).

| Capteur | x | y | z | Orientation |
| --- | ---: | ---: | ---: | --- |
| C1, centre du plan laser | 0 | 0 | 0,322 m | 0° vers l'avant (à confirmer devant un mur dans RViz) |
| Webcam gauche, centre optique | 0,442 m | +0,060 m | 0,270 m | tangage 10° vers le sol |
| Webcam droite, centre optique | 0,442 m | −0,060 m | 0,270 m | tangage 10° vers le sol |

![Mât des caméras](vues/15_mat_cameras.png)
![Profil : caméras inclinées de 10°](vues/16_mat_cameras_profil_10deg.png)
![Barre vue de dessous : emboîtement, languettes, goussets, lamages des vis photo](vues/18_barre_stereo_dessous.png)

### À mesurer sur les webcams

L'enveloppe des webcams vient des cotes commerciales : 81,3 × 48,3 × 47 mm, champ de 85° en
diagonale, soit 75° × 45°. Mesurer les cotes suivantes, les reporter dans les paramètres, puis
relancer `11` et `12` :
- `d4_cam_axe_sur_pied` (33 mm supposés) : hauteur du centre de l'objectif au-dessus de la face du
  pied qui porte le filetage ;
- `d4_cam_axe_devant_vis` (22 mm supposés) : avance de l'objectif devant l'axe du filetage.

Ces deux cotes ne changent que la longueur du profilé avant. La base de 120 mm suppose le filetage
centré sous l'objectif. La calibration mesure de toute façon la base réelle.

## Plan de perçage de la planche

[plan_percages/plan_percages.pdf](plan_percages/plan_percages.pdf) contient trois parties :
- **page 1** : plan coté au 1:2,5 ;
- **page 2** : coordonnées de chaque trou depuis le bord gauche et le bord avant, avec la
  visserie ;
- **pages 3 à 6** : **gabarit à l'échelle 1** sur 4 feuilles A4.
  - Imprimer à 100 % et vérifier la règle de 100 mm.
  - Assembler les feuilles sur les croix vertes (repères tous les 50 mm).
  - Scotcher le gabarit sur la planche et pointer au centre des croix.

![Plan de perçage](plan_percages/plan_percages.png)

| Repère | Ø | Pour |
| --- | ---: | --- |
| P1–P4 | 8,5 | Supports de pivot (vis H M8×20 par-dessous) |
| M1–M8 | 8,5 | Supports moteurs arrière, 4 par support (M8×25 traversantes) |
| S1–S4 | 3,4 | Oreilles du servo : **entraxe 48 × 10 supposé, pointer avec le servo** |
| E1–E2 | 8,5 | Équerres du profilé des caméras (Y 25 depuis le bord avant, X ±40 de l'axe) |
| E3–E4 | 8,5 | Équerres du profilé du lidar (Y 440 depuis le bord avant, X ±40 de l'axe) |

Le Ø26 que le modèle garde au centre de la découpe du servo ne se perce pas.

## Mouvement vérifié

Le servo est la liaison `D4_SERVO_DIRECTION`, bornée à **±31°**. Aucune collision.

| Servo | Roue intérieure | Roue extérieure |
| ---: | ---: | ---: |
| ±10° | 10,0° | 9,1° |
| ±20° | 21,0° | 17,4° |
| ±30° | 33,9° | 24,5° |
| ±31° | 35,4° | 25,1° |

Les butées imprimées arrêtent la fusée à environ **38,6°**.
**Dans le firmware, limiter le servo à ±31° autour du neutre.**

![Vue de dessous à 31°](vues/03_dessous_braquage_31deg.png)

## Pièces à imprimer (PETG, aucun support)

| Fichier | Qté | Taille (mm) | Orientation |
| --- | ---: | --- | --- |
| `support_pivot_D` / `_G` | 1 + 1 | 96 × 48 × 20 | Face planche sur le plateau (butées vers le haut) |
| `fusee_renforcee_D` / `_G` | 1 + 1 | 50 × 52 × 56 | Face du bras sur le plateau ; avec le gousset sous le pont (`prototype_final/scripts/19_fusees_renfort.py`) |
| `palonnier` | 1 | 37 × 53,5 × 9 | Plateau des biellettes sur le plateau, empreinte de l'étoile vers le haut |
| `biellette` | 2 | 150 × 14 × 6 | Face plane (dessus) sur le plateau |
| `rondelle_2mm` | **4** | Ø12 × 2 | À plat : une sous la tête de chaque vis-pivot et de chaque vis de roue |
| `entretoise_roue_2.2mm` | 2 | Ø12 × 2,2 | À plat ; entre les bagues intérieures des deux 608ZZ de chaque roue |
| `rondelle_precharge_1.8mm` / `_1.9mm` | 2 + 2 | Ø12 × 1,8 / 1,9 | Entre les deux 608ZZ de chaque fusée, à la place de la rondelle de 2 mm : supprime le jeu (voir « Pivot sans jeu ») |
| `rondelle_M3_0.6mm` | 8 (si pas de rondelles M3 métal) | Ø5 × Ø3,3 × 0,6 | À plat. Elle n'appuie que sur la bague intérieure des 3×10×4 ; couche de 0,2 mm, 3 couches |
| `jante_608ZZ` | 2 | Ø124 × 33 | Face extérieure (celle des trous d'inserts) sur le plateau, PETG |
| `flasque_pneu` | 2 | Ø124 × 3 | À plat, PETG |
| `pneu_TPU` | 2 | Ø150 × 31 | À plat, **TPU 90A**, gyroïde 15–20 %, 3 parois |
| `support_lidar_C1` | 1 | 56 × 56 × 98,5 | Plaque (face du lidar) sur le plateau, emboîtement vers le haut ; 4 parois, gyroïde 20 % |
| `barre_stereo` | 1 | 170 × 60 × 36 | STL **déjà couché** : dessus de la barre (appui des caméras) sur le plateau, emboîtement vers le haut ; 4 parois, gyroïde 25 % |

Ajustements dessinés :
- logements 608ZZ Ø22,15 et 3×10×4 Ø10,15 (emmanchement léger) ;
- passages Ø8,4 et Ø3,2 ;
- écrous M3 prisonniers sur le dessus du bras et du palonnier.

Imprimer d'abord **un support et une fusée** pour valider l'emmanchement et le mouvement.

## Quincaillerie

| Qté | Élément | Rôle |
| ---: | --- | --- |
| 8 | **608ZZ** | 4 aux pivots (2 par fusée) + 4 dans les roues (2 par moyeu) |
| 4 | **3×10×4** | Articulations des biellettes |
| 2 | Vis H **M8×30** + 2 écrous M8 | Pivots : vis par-dessous, écrou prisonnier dans le support (côté planche), frein-filet conseillé |
| 2 | Vis H **M8×30** + 2 écrous M8 | Axes de roue : écrou prisonnier dans la jambe de la fusée, frein-filet conseillé |
| 12 | **Inserts M3 à chaud** (Ø4,6 × 5,7) + 12 vis **M3×8** | Flasques des pneus (6 par roue) |
| 4 | Vis H **M8×20** (M8×25 confortable ; M8×30 dépasse de 11 mm) + 4 écrous M8 + 4 rondelles M8 | Supports à travers la planche de 5 mm : tête prisonnière sous le support, écrou sur le dessus (le support sert de gabarit de perçage) |
| 4 | Vis **M3×12 tête bombée** + 8 rondelles M3 + 4 écrous M3 | Biellettes : une rondelle entre chaque roulement et le bras (ou le palonnier), une sous chaque tête de vis. Sans rondelles métal, imprimer `rondelle_M3_0.6mm` (×8) |
| 4 | Vis **M3×12** + écrous et rondelles | Oreilles du servo à travers la planche de 5 mm |
| 1 | Vis **M3×16** + **rondelle large M3** (DIN 9021, Ø9 ou Ø12) | Vis centrale, par-dessous : tient le palonnier imprimé et l'étoile sur l'axe en laiton du servo (taraudé M3). Remplace la vis d'origine de l'étoile. Longueur : voir le montage |
| 2 | **Profilés 40 × 40 type B** (rainure 10) : **90 mm** (lidar) et **90 mm** (caméras) | Mâts |
| 4 | **Équerres 40 × 40** (rainure 10) | Pieds des mâts, à gauche et à droite de chaque profilé |
| 4 | Vis H **M8×20** + 4 rondelles M8 + 4 écrous M8 | Équerres sur la planche (écrou sous la planche) |
| 8 | Vis **M8×16** + 8 **écrous en T M8 rainure 10** | 4 pour les équerres sur les profilés, 2 pour la barre stéréo, 2 pour le support du lidar |
| 4 | Vis **M2,5×8** tête cylindrique + rondelles M2,5 | C1 sur son support (4 mm maxi dans le lidar) |
| 2 | Vis photo **1/4"-20 × 1/2"** (ou M6×12 si le filetage est bien M6) | Webcams sur la barre stéréo |

## Montage

1. **Planche** :
   - découper le rectangle du servo (41,4 × 21 mm) ;
   - percer tous les trous d'après le [gabarit 1:1](plan_percages/plan_percages.pdf) (supports de
     pivot, supports moteurs, servo, équerres des deux profilés). Les supports eux-mêmes peuvent servir de
     gabarit pour contrôler.
2. **Pivot**, sur l'établi :
   - glisser un écrou M8 dans l'empreinte du dessus du support ;
   - dans la fusée, presser le 608ZZ du bas jusqu'à l'arête, poser une rondelle de précharge (1,9 mm pour commencer) sur sa bague intérieure, puis presser le 608ZZ du haut jusqu'à l'arête ;
   - glisser l'écrou de roue dans sa poche (côté intérieur de la jambe) ;
   - par-dessous, passer la vis M8×30 avec une rondelle de 2 mm à travers la fusée et le support, et la visser dans l'écrou ;
   - serrer franchement, avec du frein-filet : le serrage ne porte que sur les bagues intérieures, il ne freine pas la fusée (voir « Pivot sans jeu »).
3. **Fixation** : par-dessous, glisser 2 vis H M8 dans les empreintes hexagonales du support, à travers la planche ; rondelle et écrou M8 sur le dessus. La planche ferme l'empreinte de l'écrou du pivot.
4. **Servo et palonnier** :
   - passer le servo dans la planche et visser les oreilles ;
   - encastrer l'étoile dans l'empreinte du palonnier imprimé, face plate au fond ;
   - servo au neutre, enfoncer l'étoile seule à fond sur la cannelure et mesurer de combien sa face plate dépasse le bout de l'axe en laiton (environ 4,2 mm d'après les mesures) ;
   - longueur de la vis M3 ≈ 12 mm + ce dépassement : 6,7 mm de plastique sous l'étoile, 0,8 mm de rondelle et 4 à 5 mm de filet dans l'axe. Avec 4,2 mm, une **M3×16** ; elle ne doit pas buter au fond de l'axe avant de serrer ;
   - remettre l'étoile dans le palonnier, enfoncer l'ensemble sur la cannelure, puis, par-dessous, passer la vis M3 avec la rondelle large dans l'accès central (Ø7) et la serrer dans l'axe, avec du frein-filet bleu ;
   - roues droites, monter les biellettes (M3 par-dessous : rondelle, roulement, rondelle, bras, écrou prisonnier).
5. **Roues** :
   - poser les 6 inserts M3 dans la jante ; ébavurer le dessous de l'arête centrale ;
   - presser le 608ZZ intérieur par le côté fusée jusqu'à l'arête (main ou étau, en appuyant sur
     la bague extérieure seulement) ;
   - **poser l'entretoise de 2,2 mm maintenant**, par l'autre côté, sur sa bague intérieure. Une
     fois le second roulement en place, on ne peut plus la glisser ;
   - presser le 608ZZ extérieur par le côté flasque (il passe dans l'alésage Ø24) jusqu'à l'arête ;
   - enfiler le pneu à plat, poser le flasque et serrer les 6 vis M3×8 ;
   - passer la vis M8×30 et sa rondelle de 2 mm par l'extérieur, puis la visser dans l'écrou de la
     jambe avec une douille de 13. Serrer franchement, avec du frein-filet ;
   - vérifier que la roue lancée à la main fait plusieurs tours seule, et qu'elle a un très léger
     jeu axial (0,2 mm au plus). Si elle tourne dur : arête mal ébavurée ou roulement pas au fond
     de son logement.

6. **Mâts** (lidar et caméras, même méthode) :
   - deux profilés de **90 mm** (lidar et caméras), ébavurés ;
   - poser chaque profilé debout, centré sur l'axe : caméras de Y 5 à 45 mm depuis le bord avant,
     lidar de Y 420 à 460 mm (au-dessus de l'axe des roues arrière) ;
   - une équerre à gauche et une à droite : vis M8×20 + rondelle à travers la planche (trous E),
     écrou dessous ; vis M8×16 dans un écrou en T M8 de la rainure du profilé ;
   - mettre deux écrous en T M8 dans les rainures gauche et droite, à 10 mm sous le haut du
     profilé des caméras.
7. **Lidar** (avant de poser la coque) :
   - visser le C1 sur son support : 4 M2,5×8, clé Allen de 2 par les gorges des coins ; câble vers
     l'arrière dans la gorge ;
   - le support du lidar se pose avec le C1 déjà vissé dessus. Deux écrous en T M8 sont vissés de deux
     tours sur les vis M8×16 latérales, en long. On enfile le support sur le haut du profilé : les écrous
     entrent dans les rainures gauche et droite, les languettes dans les rainures avant et arrière. On
     descend jusqu'en butée, puis on serre les deux vis.
8. **Webcams** :
   - poser chaque webcam pied à plat sur la barre, objectif vers l'avant et face avant parallèle
     au bord de la barre ;
   - serrer la vis photo par-dessous ;
   - mettre la charnière du pied dans sa position repliée (objectif dans l'axe du pied), puis la
     bloquer, par exemple d'un point de colle chaude ;
   - emboîter la barre sur le profilé avant et serrer les deux M8 ;
   - **calibrer la paire seulement quand tout est serré**, et recalibrer si une caméra a bougé.

## Pivot sans jeu

Le jeu ne se règle pas en serrant la fusée contre le support : il y a volontairement 1 mm entre
les deux, pour qu'ils ne frottent pas. Tout passe par les **bagues intérieures** des deux 608ZZ :

- **Chaîne serrée par la vis-pivot** : tête de vis → rondelle de 2 mm → bague intérieure du 608ZZ
  du bas → rondelle centrale → bague intérieure du 608ZZ du haut → portée Ø12 du support → support
  → écrou prisonnier. Les bagues extérieures, elles, sont tenues dans la fusée de part et d'autre
  de l'arête de 2 mm.
- **Précharge** : si la rondelle centrale est un peu plus fine que l'arête (1,9 ou 1,8 mm au lieu
  de 2), le serrage rapproche les deux bagues intérieures. Le jeu interne des roulements
  disparaît, axialement et en basculement. C'est le montage classique de deux roulements opposés.
- **Serrage** : serrer franchement, avec du frein-filet moyen (bleu). Le serrage ne porte que sur
  les bagues intérieures : il ne freine pas la fusée, tant que la précharge reste légère.
- **Réglage** sur l'établi, roue levée :
  - **du jeu** quand on secoue la roue de haut en bas → rondelle plus fine (1,8) ;
  - **la fusée accroche ou tourne dur** → rondelle plus épaisse (2,0) ;
  - les trois épaisseurs sont fournies ; une rondelle peut aussi être poncée.
- **Roulements bien tenus** : chaque 608ZZ doit être emmanché jusqu'à l'arête et ne pas bouger dans
  son logement. S'il entre trop facilement, une goutte de colle cyanoacrylate ou de frein-filet
  fort pour roulements (Loctite 641) sur la bague extérieure supprime ce jeu-là.
- **Vis filetée dans les roulements** : le filetage (Ø7,8 environ) laisse un peu de jeu dans
  l'alésage de 8 mm. Le serrage des bagues le neutralise ; une vis à tige lisse au droit des
  roulements est encore mieux si tu en trouves.

Le reste du jeu au volant vient du **servo** (engrenages, cannelure du palonnier) et des
**biellettes**, montées sur la même idée : bague intérieure serrée, sans frottement.

## À mesurer ou valider

- **DT996 mesuré** (15/10) : boîtier 40,4 × 20 × 39,3 mm, oreilles de 2,2 mm dont le dessous
  est à 28,5 mm de la base, bossage jusqu'à 40,8, cannelure jusqu'à 44,8, palonnier étoile monté
  jusqu'à 49 mm. L'étoile (6 branches, Ø31,4, épaisseur 2,6, 5,5 avec le moyeu côté servo) a été
  relevée sur photo au pied à coulisse.
  - **À confirmer : les 49 mm avec l'étoile enfoncée à fond et sa vis serrée.** Si la cote change,
    modifier `d4_etoile_sous_oreilles` (= cote − 30,7) : l'empreinte et le rebord suivent.
  - Restent à relever : le diamètre du bossage (12 mm supposés), la longueur hors tout des
    oreilles (54) et l'entraxe de leurs trous (48 × 10).
  - Imprimer d'abord le palonnier seul et essayer l'étoile dans l'empreinte.
  - Fixation : vis M3 centrale dans l'axe en laiton (taraudé M3, confirmé par Hugo). Les trous de
    l'étoile n'acceptent que du M1 ; les trois passages M2 du palonnier restent inutilisés.
- **Emmanchements** des roulements sur un premier tirage.
- **Pneu TPU 90A** : serrage sur la jante (1 mm), remplissage et montage avec le flasque à valider sur un premier tirage (`d4_pneu_serrage`).
- **Charge réelle et couple** : les poses vérifiées ne valident pas la résistance.

## Scripts

Ils passent par le serveur MCP local de Fusion, dont l'adresse change à chaque redémarrage :
utiliser la variable `FUSION_MCP_URL`. Ils réutilisent `../direction_v03_simple/scripts/d3_helpers.py`.

| Script | Rôle |
| --- | --- |
| `01_construire.py` | Paramètres, pièces, liaisons, planche avec découpe servo |
| `02_verifier.py` | Balayage du servo, collisions, essais de paramètres, hauteurs → JSON |
| `03_exporter.py` | STL des pièces (barre stéréo couchée pour l'impression), STEP + F3D du groupe, manifeste |
| `04_portee_roue.py` | Portée Ø12 (bague intérieure) sur la face extérieure des fusées |
| `05_roues.py` | Jantes à rayons, pneus TPU à crampons trapézoïdaux, rondelles et vis de roue |
| `06_planche_30cm.py` | Écartement lié à la largeur de planche, jeu pneu/planche en braquage |
| `07_supports_moteurs.py` | Planche 30 × 50, import et placement des supports moteurs arrière |
| `08_planche_5mm.py` | Planche de 5 mm, modifiée en place (garde les congés ajoutés à la main sur les fusées) ; sa partie servo/palonnier est remplacée par le script 10 |
| `09_supports_M8.py` | Supports de pivot fixés par 2 vis M8 (tête prisonnière), en place |
| `10_servo_mesure_etoile.py` | Servo aux cotes mesurées, palonnier étoile encastré, découpe 41,4 × 21, en place |
| `11_lidar_stereo.py` | Deux profilés 40 × 40 et leurs équerres, support lidar, C1 (STEP Slamtec), barre stéréo, webcams, perçages de la planche ; réexécutable |
| `12_verifier_capteurs.py` | Collisions (direction au neutre et à ±31°), champ des webcams et plan laser par rayons, relevé des trous → `verification_capteurs.json`, `plan_percages.json` |
| `13_plan_percages.py` | Hors Fusion : plan de perçage PDF (plan, coordonnées, gabarit 1:1) à partir de `plan_percages.json` |
| `14_verifier_roues.py` | Jeux entre ce qui tourne avec la roue et ce qui reste fixe, direction au neutre et à ±31°, empilage sur l'axe → `verification_roues.json` |
| `15_jante_flasque.py` | Jantes : flasque extérieur vissé (6 inserts M3), arête Ø19,5, chanfrein d'entrée du pneu ; en place, réexécutable |
| `16_roue_annexes.py` | Hors Fusion : entretoise de roue 2,2 mm (STL) et demi-coupe du moyeu |
| `fusion_mcp.py` | Client du serveur MCP de Fusion (hors Fusion) : `run`, `shot`, `read` |

Le document Fusion n'est pas enregistré par les scripts.

**Planche de 5 mm** : elle porte les supports de pivot, le servo et les supports moteurs. Une
planche aussi fine peut fléchir sous la batterie et l'électronique ; si c'est le cas, un tasseau
vissé dessous entre les supports moteurs et les pivots suffit à la raidir.
