#!/usr/bin/env bash
# Préparation unique du Raspberry Pi (à lancer SUR le Pi, avec sudo pour apt et les groupes).
#   ssh pi-wifi 'bash -s' < installer_pi.sh
set -euo pipefail

sudo apt-get update
sudo apt-get install -y python3-numpy python3-scipy python3-opencv python3-serial \
  python3-aiohttp alsa-utils

# Caméras, ports série, micro et I2C sans root
sudo usermod -aG video,dialout,audio,i2c "$USER"

# GPS sur l'UART du GPIO : console série coupée, UART activé (déjà fait le 18/10, sans effet sinon)
sudo raspi-config nonint do_serial_cons 1 || true
grep -q '^enable_uart=1' /boot/firmware/config.txt || echo 'enable_uart=1' | sudo tee -a /boot/firmware/config.txt

# Le service utilisateur démarre au boot sans session ouverte
sudo loginctl enable-linger "$USER"

python3 - <<'PY'
import sys, numpy, scipy, cv2, aiohttp, serial
print("Python", sys.version.split()[0], "| numpy", numpy.__version__, "| scipy", scipy.__version__,
      "| OpenCV", cv2.__version__, "| aiohttp", aiohttp.__version__)
PY
echo "Fait. Se reconnecter (groupes), puis lancer deployer.sh depuis le PC."
