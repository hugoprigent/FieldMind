#!/usr/bin/env bash
# Installation ou mise à jour de la démo sur un VPS Ubuntu 24.04 (à lancer en root).
#   sudo ./installer.sh demo.exemple.fr
# Le code doit déjà être copié dans /opt/fieldmind/software (fait par le workflow de déploiement).
# Idempotent : relancer le script redéploie sans rien casser.
set -euo pipefail

DOMAINE="${1:?usage : installer.sh <domaine>}"
RACINE=/opt/fieldmind/software
ICI="$(cd "$(dirname "$0")" && pwd)"

echo "==> Paquets système"
if ! command -v caddy >/dev/null; then
    apt-get update -qq
    apt-get install -y -qq debian-keyring debian-archive-keyring apt-transport-https curl gpg
    curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/gpg.key' \
        | gpg --dearmor -o /usr/share/keyrings/caddy-stable-archive-keyring.gpg
    curl -1sLf 'https://dl.cloudsmith.io/public/caddy/stable/debian.deb.txt' \
        > /etc/apt/sources.list.d/caddy-stable.list
    apt-get update -qq
fi
apt-get install -y -qq python3.12 python3.12-venv caddy rsync libosmesa6

echo "==> Utilisateur de service"
id fieldmind >/dev/null 2>&1 || useradd --system --home /opt/fieldmind --shell /usr/sbin/nologin fieldmind
install -d -o fieldmind -g fieldmind "$RACINE/logs" /var/cache/fieldmind

echo "==> Environnement Python"
[ -d "$RACINE/.venv" ] || python3.12 -m venv "$RACINE/.venv"
"$RACINE/.venv/bin/pip" install -q --upgrade pip
"$RACINE/.venv/bin/pip" install -q -r "$RACINE/requirements-demo.txt"
chown -R fieldmind:fieldmind /opt/fieldmind

echo "==> Réglages (jamais écrasés s'ils existent)"
install -d -m 750 -o root -g fieldmind /etc/fieldmind
if [ ! -f /etc/fieldmind/demo.env ]; then
    install -m 640 -o root -g fieldmind "$RACINE/.env.example" /etc/fieldmind/demo.env
    echo "    /etc/fieldmind/demo.env créé depuis .env.example : y mettre la clé d'API."
fi

echo "==> Service et proxy"
install -m 644 "$ICI/fieldmind-demo.service" /etc/systemd/system/fieldmind-demo.service
sed "s/__DOMAINE__/$DOMAINE/" "$ICI/Caddyfile.modele" > /etc/caddy/Caddyfile
install -d -o caddy -g caddy /var/log/caddy
systemctl daemon-reload
systemctl enable --now fieldmind-demo.service
systemctl restart fieldmind-demo.service
systemctl reload caddy || systemctl restart caddy

echo "==> Vérification"
for _ in $(seq 1 30); do
    if curl -fsS http://127.0.0.1:8000/api/sante >/dev/null; then
        curl -fsS http://127.0.0.1:8000/api/sante; echo
        echo "Démo en ligne : https://$DOMAINE/"
        exit 0
    fi
    sleep 1
done
echo "Le service ne répond pas : journalctl -u fieldmind-demo -n 50" >&2
exit 1
