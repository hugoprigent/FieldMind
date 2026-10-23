[Français](README.md) · **English**

# FieldMind inference on the ASUS Ascent GX10

These files configure the AI services of the rover's `gx10` profile. As of 23 October 2026, eight models are served on the GX10's GB10. Qwen3.6, Qwen3-ASR and RoboBrain 2.5 have been checked with real calls; the others are connected to the rover and checked in simulation (detailed status in [MODELES.md](../../MODELES.en.md)). This file describes their configuration; it is not a benchmark.

| Service | Port | Model | Role |
| --- | --- | --- | --- |
| `modele` | 8000 | `nvidia/Qwen3.6-35B-A3B-NVFP4` | Agent, images and tool calls |
| `transcription` | 8001 | `Qwen/Qwen3-ASR-1.7B` | Speech recognition |
| `robotique` | 8002 | `BAAI/RoboBrain2.5-4B` | Object pointing |
| `outils` | 8003 | D-FINE, Depth Anything V2 (metric), SAM 2.1, Qwen3-VL-Embedding-2B | Detection, depth, segmentation, embeddings |
| `voix` | 8004 | `Qwen/Qwen3-TTS-12Hz-0.6B-CustomVoice` | Speech synthesis |

The exact identifiers and settings are in [`docker-compose.yml`](docker-compose.yml) and [`config/ia.toml`](../../config/ia.toml). The weights are not distributed. Check their licenses and the publisher's recipes before downloading or changing a model.

## Starting the main model

Prerequisites on the GX10: a working NVIDIA driver, Docker with the NVIDIA Container Toolkit, access to the registries and enough space for the weights. These files use the `vllm/vllm-openai:v0.31.0` image.

```bash
cd software/rover/deploiement/gx10
cp .env.exemple .env
# Set VLLM_API_KEY to a random personal key, keep .env private.
docker compose up -d modele
docker compose logs -f modele
```

The initial download and compilation can take a long time. The complementary services are started with `docker compose up -d --build`, after checking their compatibility and the available memory; they are ordered by dependencies to avoid simultaneous loads at startup.

## Connecting the rover

The ports bind to **127.0.0.1 by default**, and the Docker configuration requires a non-empty key. For remote access, use a private network or an SSH tunnel. If the rover must reach a private interface of the GX10 directly, set `FIELDMIND_BIND_IP` to the address of that interface and apply suitable access rules.

In `software/rover/.env`, on the station or the Pi:

```dotenv
FIELDMIND_GX10_HOTE=gx10.example.internal
GX10_API_KEY=REMPLACER_PAR_LA_MEME_CLE_PRIVEE
FIELDMIND_IA_PROFIL=gx10
```

`gx10.example.internal` is an example to be replaced, and `REMPLACER_PAR_LA_MEME_CLE_PRIVEE` stands for the same private key. No lab account or address is included in this edition. On the same network, `gx10.local` may also work if mDNS is configured.

## Checking

From `software/rover` with the services available:

```bash
.venv/bin/python deploiement/gx10/verifier.py --profil gx10
.venv/bin/python deploiement/gx10/verifier.py --profil gx10 --wav essai.wav
```

The script contacts real models and checks availability, text response, function calls, pointing on a test image and transcription. A missing service can make some checks fail. No test in this command moves the rover. The project's unit tests use fake servers and must not be presented as GX10 benchmarks.
