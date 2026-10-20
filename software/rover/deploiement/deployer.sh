#!/usr/bin/env bash
# Copie le logiciel du rover sur le Pi, installe le service utilisateur fieldmind-rover et le
# (re)démarre. Usage, depuis software/rover/deploiement : ./deployer.sh [hôte ssh, défaut pi-wifi]
#
# Le service remplace la supervision fieldmind-embarque : les deux ouvriraient les mêmes ports
# série et caméras. Celle-ci est donc arrêtée et désactivée (réactivable :
# systemctl --user enable --now fieldmind-embarque).
set -euo pipefail
hote="${1:-pi-wifi}"
racine="$(cd "$(dirname "$0")/.." && pwd)"

tar czf - -C "$racine" --exclude=__pycache__ --exclude=.venv --exclude=.pytest_cache \
  --exclude=.ruff_cache --exclude=.env fieldmind config deploiement/fieldmind-rover.service \
  | ssh "$hote" 'mkdir -p ~/fieldmind/rover && tar xzf - -C ~/fieldmind/rover'

# Clés des modèles (GEMINI_API_KEY…) : fichier local software/rover/.env, jamais dans git
if [ -f "$racine/.env" ]; then
  ssh "$hote" 'mkdir -p ~/.config/fieldmind && umask 077 && cat > ~/.config/fieldmind/env' \
    < "$racine/.env"
fi

ssh "$hote" bash -s <<'DISTANT'
set -e
export LC_ALL=C.UTF-8
# Environnement Python sans sudo : paquets apt du système + ceux qui manquent (scipy, aiohttp)
if [ ! -x ~/fieldmind/venv/bin/python ]; then
  python3 -m venv --system-site-packages ~/fieldmind/venv
fi
~/fieldmind/venv/bin/python -c 'import scipy, aiohttp' 2>/dev/null \
  || ~/fieldmind/venv/bin/pip install -q --disable-pip-version-check scipy aiohttp
mkdir -p ~/.config/systemd/user
if systemctl --user is-enabled --quiet fieldmind-embarque 2>/dev/null; then
  systemctl --user disable --now fieldmind-embarque
  echo "supervision fieldmind-embarque arrêtée (ports libérés)"
fi
cp ~/fieldmind/rover/deploiement/fieldmind-rover.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable --quiet fieldmind-rover
systemctl --user restart fieldmind-rover
sleep 3
systemctl --user --no-pager --lines=0 status fieldmind-rover | head -3
ip -4 -o addr show wlan0 | awk '{print "poste : http://" substr($4, 1, index($4, "/") - 1) ":8080"}'
DISTANT
# Journal : ssh pi-wifi journalctl _SYSTEMD_USER_UNIT=fieldmind-rover.service -f -o cat
