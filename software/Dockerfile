# Image de la démo FieldMind : serveur, cerveau et simulateur (processeur seul, sans GPU).
#   docker build -t fieldmind-demo software
#   docker run --rm -p 8000:8000 -e OPENROUTER_API_KEY=... fieldmind-demo
# Sans clé : chat par mots-clés, vision par le détecteur parfait de la simulation.
FROM python:3.12-slim

# OSMesa : rendu hors écran de la caméra simulée, sans GPU ni serveur graphique.
RUN apt-get update \
    && apt-get install -y --no-install-recommends libosmesa6 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements-demo.txt .
RUN pip install --no-cache-dir -r requirements-demo.txt

COPY config config
COPY services services
COPY simulation simulation
COPY interface interface

ENV MUJOCO_GL=osmesa \
    PYTHONUNBUFFERED=1 \
    NUMBA_CACHE_DIR=/tmp/numba \
    PORT=8000
RUN useradd --create-home --uid 1000 fieldmind && mkdir -p logs && chown fieldmind logs
USER fieldmind
EXPOSE 8000
HEALTHCHECK --interval=30s --timeout=5s CMD python -c "import urllib.request,os; urllib.request.urlopen(f'http://127.0.0.1:{os.environ[\"PORT\"]}/api/sante')"
CMD ["sh", "-c", "exec uvicorn services.api.serveur:app --host 0.0.0.0 --port ${PORT} --proxy-headers --forwarded-allow-ips='*'"]
