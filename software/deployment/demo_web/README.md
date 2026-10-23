**Français** · [English](README.en.md)

# Mise en ligne de la démo

Architecture : Caddy (HTTPS automatique, WebSocket) → uvicorn (un seul processus, sessions en
mémoire) sur `127.0.0.1:8000`, service systemd `fieldmind-demo` durci.

## Première mise en ligne

1. **VPS** : Ubuntu 24.04, 2 vCPU et 4 Go de RAM suffisent pour les 10 sessions simultanées
   autorisées (chaque session : physique ≈ 4 %, cerveau ≈ 6 % d'un cœur).
2. **DNS** : un enregistrement A (et AAAA) du domaine vers le VPS ; ports 80 et 443 ouverts.
3. **Compte de déploiement** sur le VPS, avec une clé SSH dédiée et `sudo` sans mot de passe
   limité au script d'installation, par exemple dans `/etc/sudoers.d/fieldmind` :
   `deploiement ALL=(root) NOPASSWD: /opt/fieldmind/software/deployment/demo_web/installer.sh, /usr/bin/install`
4. **Secrets GitHub** (Settings → Secrets and variables → Actions, environnement `demo`) :
   `VPS_HOTE`, `VPS_UTILISATEUR`, `VPS_CLE_SSH`, `VPS_EMPREINTE_SSH`, `DEMO_DOMAINE`.
5. **Lancer** le workflow « Déployer la démo web » (onglet Actions).
6. **Clé d'API** (facultative) : sur le VPS, compléter `/etc/fieldmind/demo.env`
   (`FIELDMIND_LLM_PROVIDER=anthropic`, `FIELDMIND_LLM_MODEL`, `FIELDMIND_LLM_API_KEY`), puis
   `sudo systemctl restart fieldmind-demo`. Sans clé, le chat utilise l'analyseur de mots-clés.
   Plafonner la dépense de la clé dans la console du fournisseur.

## Redéployer

Relancer le workflow. Le script est idempotent ; le fichier `/etc/fieldmind/demo.env` n'est jamais
écrasé.

## Sans GitHub Actions

```bash
rsync -az --delete --exclude .venv --exclude logs software/ vps:/opt/fieldmind/software/
ssh vps 'sudo /opt/fieldmind/software/deployment/demo_web/installer.sh demo.exemple.fr'
```

## Exploitation

- État : `https://<domaine>/api/sante` (sessions ouvertes, robot distant connecté).
- Journaux : `journalctl -u fieldmind-demo -f` ; événements des sessions en JSONL dans
  `/opt/fieldmind/software/logs/`.
- Limites : 10 sessions, fermeture après 10 min d'inactivité, 30 consignes par session,
  500 appels au modèle par jour, 60 messages par seconde et par onglet.
