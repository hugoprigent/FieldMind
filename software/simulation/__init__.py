"""Simulateur du rover FieldMind (physique MuJoCo, capteurs, firmware du MCU, forêt)."""

import ctypes.util
import os

# Rendu hors écran de la caméra simulée : OSMesa (processeur) fonctionne partout, sans GPU ni
# serveur graphique. Sur une machine avec GPU, MUJOCO_GL=egl est plus rapide. À fixer avant le
# premier `import mujoco` ; seulement si la bibliothèque est installée, sinon MuJoCo ne se charge
# plus du tout (la caméra est alors désactivée, le reste du simulateur fonctionne).
if "MUJOCO_GL" not in os.environ and ctypes.util.find_library("OSMesa"):
    os.environ["MUJOCO_GL"] = "osmesa"
