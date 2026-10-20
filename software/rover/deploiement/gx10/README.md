# Inférence FieldMind sur ASUS Ascent GX10

Ces fichiers configurent les services IA du profil `gx10` du rover. Le modèle principal `nvidia/Qwen3.6-35B-A3B-NVFP4` a répondu à de vrais appels sur le GB10 pendant la session ASUS. Les autres services sont en intégration : ce fichier décrit leur configuration, pas un résultat de validation complet.

| Service | Port | Rôle prévu |
| --- | --- | --- |
| `modele` | 8000 | Agent, images et appels d'outils |
| `transcription` | 8001 | Reconnaissance vocale |
| `robotique` | 8002 | Pointage d'objets |
| `outils` | 8003 | Détection, profondeur, segmentation, empreintes |
| `voix` | 8004 | Synthèse vocale |

Les identifiants et réglages précis sont dans [`docker-compose.yml`](docker-compose.yml) et [`config/ia.toml`](../../config/ia.toml). Les poids ne sont pas distribués. Vérifier leurs licences et les recettes de l'éditeur avant téléchargement ou changement de modèle.

## Démarrer le modèle principal

Prérequis sur le GX10 : pilote NVIDIA fonctionnel, Docker avec NVIDIA Container Toolkit, accès aux registres et assez d'espace pour les poids. Ces fichiers utilisent l'image `vllm/vllm-openai:v0.31.0` ; les réglages des modèles complémentaires restent à valider individuellement.

```bash
cd software/rover/deploiement/gx10
cp .env.exemple .env
# Renseigner VLLM_API_KEY avec une clé personnelle aléatoire, garder .env privé.
docker compose up -d modele
docker compose logs -f modele
```

Le téléchargement et la compilation initiaux peuvent être longs. Les services complémentaires se lancent avec `docker compose up -d --build`, après vérification de leur compatibilité et de la mémoire disponible ; ils sont ordonnés par dépendances pour éviter les charges simultanées au démarrage.

## Relier le rover

Les ports se lient à **127.0.0.1 par défaut**, et une clé non vide est exigée par la configuration Docker. Pour un accès distant, utiliser un réseau privé ou un tunnel SSH. Si le rover doit joindre directement une interface privée du GX10, définir `FIELDMIND_BIND_IP` avec l'adresse de cette interface et appliquer les règles d'accès adaptées.

Dans `software/rover/.env`, sur le poste ou le Pi :

```dotenv
FIELDMIND_GX10_HOTE=gx10.example.internal
GX10_API_KEY=REMPLACER_PAR_LA_MEME_CLE_PRIVEE
FIELDMIND_IA_PROFIL=gx10
```

`gx10.example.internal` est un exemple à remplacer. Aucun compte ou adresse du laboratoire n'est inclus dans cette édition. Sur un même réseau, `gx10.local` peut aussi fonctionner si mDNS est configuré.

## Vérifier

Depuis `software/rover` avec les services disponibles :

```bash
.venv/bin/python deploiement/gx10/verifier.py --profil gx10
.venv/bin/python deploiement/gx10/verifier.py --profil gx10 --wav essai.wav
```

Le script contacte de vrais modèles et vérifie disponibilité, réponse texte, appels de fonctions, pointage sur une image d'essai et transcription. Une absence de service peut faire échouer certaines vérifications. Aucun test de cette commande n'actionne le rover. Les tests unitaires du projet utilisent des faux serveurs et ne doivent pas être présentés comme des benchmarks du GX10.
