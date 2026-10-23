# Logiciels FieldMind

## Rover actuel

Le logiciel utilisé avec le prototype est dans [`rover/`](rover/README.md) : Raspberry Pi, pilotes, navigation, missions, IA, poste web et simulation du matériel. Le firmware est dans [`esp32/esp_c5/`](esp32/esp_c5/README.md). Commencer par le [README principal](../README.md).

## Première démo web — MuJoCo

Cette pile est la première démo MuJoCo du projet. Elle illustre la navigation dans une forêt simulée et possède ses propres services et interface. Elle est distincte de la pile du rover courant ; la [simulation publique](https://hugoprigent.com/fieldmind/) sert désormais le poste du rover (`software/rover/fieldmind/public_demo.py`).

Python 3.12 et, sous Linux, OSMesa (`sudo apt install libosmesa6`) :

```bash
cd software
uv venv --python 3.12 .venv
uv pip install --python .venv/bin/python -r requirements-dev.txt
.venv/bin/uvicorn services.api.serveur:app --host 127.0.0.1 --port 8000
```

Ouvrir http://127.0.0.1:8000/. Sous Linux sans écran, définir `MUJOCO_GL=osmesa`. Sans clé d'API, la carte et la navigation simulée restent accessibles ; les consignes par modèle ne sont pas disponibles. La perception fondée sur la vérité terrain est identifiée comme simulée.

```bash
# Tests rapides de cette pile
.venv/bin/python -m pytest -q -m "not lent and not navigateur"
node --test "tests/js/**/*.test.js"
```

Une image Docker est décrite dans [`Dockerfile`](Dockerfile). Les configurations d'hébergement sont dans [`deployment/demo_web/`](deployment/demo_web/README.md). La publication de ce code ne modifie pas le service déjà en ligne.

## Études ROS 2

[`ros2/`](ros2/README.md) conserve les premiers essais ROS 2 / Gazebo et leurs outils. Le pont ROS 2 raccordé au logiciel courant est dans [`rover/ros2/`](rover/ros2/README.md). Les anciennes descriptions de capacités ne valent pas validation sur le prototype physique.
