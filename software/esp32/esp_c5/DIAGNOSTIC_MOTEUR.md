# Le moteur tourne au potentiomètre mais ignore l'ESP

Conserver la combinaison phases/Hall qui permet actuellement la rotation.
Le premier contrôle isole l'ESP : le moteur et le driver n'interviennent pas.

## 1. Vérifier la sortie électrique de l'ESP

1. Débrancher la batterie, puis l'USB.
2. Débrancher les **quatre fils entre ESP et driver** : D0, D1, D10 et GND.
   Garder la batterie débranchée pendant toute cette mesure.
3. Téléverser l'environnement PlatformIO **`test_moteur`**, puis ouvrir le
   moniteur à 115200 bauds. Seul l'ESP est alimenté en USB.
4. Régler le multimètre en **tension continue V⎓**, cordons sur COM et V.
   Mettre la pointe noire sur **GND de l'ESP**, la rouge sur **D0 / GPIO1**.
   Ne pas confondre D0 avec D1 / GPIO0.
5. Envoyer chaque commande ci-dessous avec Entrée et attendre une seconde.
   Les nombres restent appliqués jusqu'à `stop` ou `0`.

| Commande | Tension attendue entre D0 et GND |
| --- | --- |
| `0` | Environ 0 V |
| `100` | Environ 3,3 V, niveau haut constant |
| `50` | Souvent environ 1,65 V en mesure continue, moyenne du PWM |
| `stop` | Retour à environ 0 V |

La mesure à 50 % dépend du multimètre : elle ne valide pas à elle seule la
forme des impulsions. Les niveaux constants `0` et `100` sont prioritaires.
**Ne jamais faire cette séquence à 100 % avec le moteur alimenté et relié
à la commande.** Terminer par `stop`, puis débrancher l'USB avant tout recâblage.

## 2. Lire le diagnostic logiciel

Pendant la mesure isolée, après `100`, saisir **`diag`** avant `stop`.
La commande affiche :

- le nom et la date de compilation du firmware ;
- le PWM demandé et le rapport cyclique relu dans le périphérique PWM ;
- la fréquence relue, attendue proche de 5000 Hz lorsque le PWM est non nul ;
- le nombre de commandes annulées par détection de perte USB ;
- le motif logiciel de l'état courant et l'activité de la tâche moteur.

À 0 %, l'API Arduino renvoie une fréquence de 0 Hz même si le générateur
est attaché : ce résultat seul n'est pas une panne. Le registre à 100 %
doit indiquer 100 %, et le PWM reste à 100 % jusqu'à une autre commande.
La lecture des registres ne remplace pas la mesure sur D0.

| Résultat | Où chercher ensuite |
| --- | --- |
| `diag` est inconnu | Mauvais environnement ou ancien firmware téléversé |
| PWM annulé et compteur USB augmenté | Liaison USB ou détection USB côté logiciel ; conserver le diagnostic avant de relancer |
| Frein demandé ou défaut affiché | Motif logiciel indiqué par `etat` / `diag` |
| Registre à 100 %, mais D0 ne monte pas vers 3,3 V, ESP isolé | Broche mesurée, configuration GPIO/PWM ou carte ESP ; les Hall et le driver sont hors circuit |
| D0 passe bien de 0 à environ 3,3 V | La sortie ESP répond ; vérifier ensuite l'interface électrique avec le driver |

## 3. Vérifier l'interface du driver, une fois l'étape 1 réussie

Un fil correctement placé ne garantit pas que le signal convient à l'entrée.
Identifier l'inscription exacte de l'entrée : `VR / 0–5V` ou `P / PWM`, ainsi
que les ponts de sélection présents sur la révision reçue. Garder le
potentiomètre au minimum, position d'arrêt déjà observée.

Une mise en œuvre documentée du ZS-X11D1 distingue l'entrée analogique et
l'entrée PWM `P`. Pour cette dernière, elle indique des impulsions de
2,5 à 5 V, et un pont J1 à fermer sur **cette révision**. Cette configuration
n'est pas une instruction pour ponter une carte différente sans l'identifier.
Voir la [description illustrée, pages 20–22](https://thesis.unipd.it/retrieve/dfe158de-ec8a-41d3-a9b9-6854970fbf38/Gervasio_Danilo.pdf).

Le code actuel génère des impulsions 0/3,3 V à 5 kHz. Il ne génère pas une
tension analogique couvrant 0–5 V. Selon l'entrée identifiée, la correction
sera la sélection de l'entrée PWM appropriée, l'adaptation des niveaux ou
une conversion PWM vers une tension analogique. Ce choix nécessite la
révision réelle de la carte ; augmenter le pourcentage ne corrige pas un
signal incompatible.

L'arrêt matériel et le phasage ne sont pas validés par ces seules mesures.
