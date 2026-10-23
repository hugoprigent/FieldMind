#!/usr/bin/env bash
# État du Pi avant installation — LECTURE SEULE, ne modifie rien.
# Usage, depuis software/rover/deploiement : ./verifier_pi.sh utilisateur@adresse-du-pi
set -uo pipefail
if [ "$#" -ne 1 ]; then
  echo "Usage : $0 utilisateur@adresse-du-pi" >&2
  exit 2
fi
hote="$1"
ssh -o ConnectTimeout=8 "$hote" bash -s <<'DISTANT'
titre() { printf '\n== %s\n' "$1"; }
titre "Système"
uname -a; cat /etc/os-release | grep PRETTY; python3 --version
vcgencmd get_throttled 2>/dev/null; vcgencmd measure_temp 2>/dev/null
df -h / | tail -1; free -h | head -2
titre "Paquets Python (apt)"
python3 - <<'PY'
import importlib
for nom in ("numpy", "scipy", "cv2", "aiohttp", "serial"):
    try:
        m = importlib.import_module(nom)
        print(f"  {nom:8s} {getattr(m, '__version__', '?')}")
    except ImportError:
        print(f"  {nom:8s} ABSENT")
PY
command -v arecord >/dev/null && echo "  arecord présent" || echo "  arecord ABSENT"
titre "Groupes de $USER"; id -nG
titre "Ports série"; ls -l /dev/ttyACM* /dev/ttyUSB* /dev/ttyS0 2>/dev/null
for p in /dev/ttyACM* /dev/ttyUSB*; do
  [ -e "$p" ] && echo "  $p : $(udevadm info -q property -n "$p" 2>/dev/null | grep -E 'ID_VENDOR_ID|ID_MODEL=' | tr '\n' ' ')"
done
titre "Caméras"; ls /dev/v4l/by-path/ 2>/dev/null | grep index0
titre "Micros (ALSA)"; arecord -l 2>/dev/null | grep -E "^carte|^card"
titre "UART du GPS"; systemctl is-active serial-getty@ttyS0 2>/dev/null; grep -E "^enable_uart" /boot/firmware/config.txt
titre "Services FieldMind"
systemctl --user is-active fieldmind-embarque fieldmind-rover 2>/dev/null | paste -sd' '
systemctl is-active fieldmind-wifi 2>/dev/null
ls ~/fieldmind 2>/dev/null
titre "Réseau"; ip -4 -o addr show wlan0 | awk '{print $4}'; ping -c1 -W2 generativelanguage.googleapis.com >/dev/null 2>&1 && echo "  Internet : oui" || echo "  Internet : non"
DISTANT
