[Français](README.md) · **English**

<a id="mise-en-ligne-de-la-démo"></a>
# Deploying the demo

Architecture: Caddy (automatic HTTPS, WebSocket) → uvicorn (single process, in-memory sessions) on `127.0.0.1:8000`, hardened systemd service `fieldmind-demo`.

<a id="première-mise-en-ligne"></a>
## Initial deployment

1. **VPS**: Ubuntu 24.04, 2 vCPUs and 4 GB of RAM are sufficient for the 10 concurrent sessions allowed (each session: physical ≈ 4 %, brain ≈ 6 % of a core).
2. **DNS**: an A (and AAAA) record for the domain pointing to the VPS; ports 80 and 443 open.
3. **Deployment account** on the VPS, with a dedicated SSH key and `sudo` passwordless access limited to the installation script, for example in `/etc/sudoers.d/fieldmind`: `deploiement ALL=(root) NOPASSWD: /opt/fieldmind/software/deployment/demo_web/installer.sh, /usr/bin/install`
4. **GitHub Secrets** (Settings → Secrets and variables → Actions, environment `demo`): `VPS_HOTE`, `VPS_UTILISATEUR`, `VPS_CLE_SSH`, `VPS_EMPREINTE_SSH`, `DEMO_DOMAINE`.
5. **Run** the "Deploy web demo" workflow (Actions tab).
6. **API key** (optional): on the VPS, complete `/etc/fieldmind/demo.env` (`FIELDMIND_LLM_PROVIDER=anthropic`, `FIELDMIND_LLM_MODEL`, `FIELDMIND_LLM_API_KEY`), then `sudo systemctl restart fieldmind-demo`. Without a key, the chat uses the keyword analyzer. Cap the key's spending in the provider's console.

<a id="redéployer"></a>
## Redeploy

Rerun the workflow. The script is idempotent; the `/etc/fieldmind/demo.env` file is never overwritten.

<a id="sans-github-actions"></a>
## Without GitHub Actions

```bash
rsync -az --delete --exclude .venv --exclude logs software/ vps:/opt/fieldmind/software/
ssh vps 'sudo /opt/fieldmind/software/deployment/demo_web/installer.sh demo.exemple.fr'
```

<a id="exploitation"></a>
## Operations

- Status: `https://<domaine>/api/sante` (open sessions, remote robot connected).
- Logs: `journalctl -u fieldmind-demo -f`; session events in JSONL format in `/opt/fieldmind/software/logs/`.
- Limits: 10 sessions, closure after 10 min of inactivity, 30 instructions per session, 500 model calls per day, 60 messages per second and per tab.
