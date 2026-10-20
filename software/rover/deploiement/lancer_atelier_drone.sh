#!/usr/bin/env bash
# Atelier entièrement local ; ne déploie rien et n'ouvre aucun matériel physique.
set -euo pipefail
racine="$(cd "$(dirname "$0")/.." && pwd)"
cd "$racine"
if [ ! -x .venv/bin/python ]; then
  echo "Environnement Python du rover absent : voir software/rover/README.md" >&2
  exit 1
fi
export FIELDMIND_IA_PROFIL=aucun
export FIELDMIND_DOSSIER="${FIELDMIND_ATELIER_DOSSIER:-$racine/.donnees-atelier}"
exec .venv/bin/python -m fieldmind --materiel simulation --hote 127.0.0.1 --port "${1:-8096}" --no-enregistrer
