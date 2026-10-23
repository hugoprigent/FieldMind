[Français](MODELES.md) · **English**

<a id="modèles-dia-du-rover--gemini-maintenant-open-source-ensuite"></a>
# Rover AI Models: Gemini for now, open source later

Each AI function is a **role** with a fixed interface. The profile selected in the cockpit (Settings > AI) determines who serves it: the Gemini API today, open source models tomorrow (GX10, the operator's Mac, or the Pi for smaller setups). The rest of the rover only sees the role.

**No role is in the safety loop.** A model can add caution (slowing down in front of a detected person), but never remove it; if it is missing or fails, the rover drives, avoids obstacles, and stops exactly as it would without it.

<a id="correspondance-des-rôles"></a>
## Role Mapping

| Role | What it does | Gemini (profile `gemini`) | GX10 (`gx10`) | Mac M3 16 Go (`mac`) | Pi 4 (`pi`) |
|---|---|---|---|---|---|
| `agent` | command → actions (function calling) | `gemini-4-argon` | Qwen3.6-35B-A3B NVFP4 (vLLM) | Gemma 4 E4B or Qwen3 8B, 4 bits (Ollama) | — |
| `vision` | describe, point to an object (`[y, x]` 0–1000), bounding boxes | `gemini-robotics-er-2-preview` | Qwen3.6-35B-A3B (same server, pointing `[x, y]`) | Moondream 2 / Qwen2.5-VL 3B (Ollama) | — |
| `pointage` | embodied model: point to an object, a zone (`chercher`) | (vision) | RoboBrain 2.5-4B (BAAI) | — | — |
| `transcription` | speech → text | `gemini-3.5-transcribe` | Qwen3-ASR-1.7B (or Whisper large-v3-turbo) | Whisper small (whisper.cpp, Metal) | Whisper base (whisper.cpp), slow |
| `synthese` | text → rover voice | `gemini-3.8-flash-tts` | Qwen3-TTS-0.6B | Piper | Piper |
| `detection` | continuous objects (people, animals, vehicles) | ER 2 at low rate (0,2 Hz) | D-FINE xlarge (0,4 Hz) | YOLO nano/small (ONNX) | YOLO nano 320 px, ~3 i/s |
| `profondeur` | single camera depth (completes stereo); SAM segmentation 2.1 | — | Depth Anything V2 metric + SAM 2.1 | Depth Anything V2 Small (ONNX) | — |
| `parole` | detect speech in the microphone | built-in detector | Silero VAD | Silero VAD | Silero VAD (< 1 ms per block) |
| `eveil` | wake word ("rover") | — | openWakeWord | openWakeWord | openWakeWord |
| `memoire` | image embeddings (recognize a place, "an object like that") | `gemini-embedding-2-preview` | Qwen3-VL-Embedding-2B | SigLIP 2 base (ONNX) | — |

Online, all models from the Gemini API are usable: the profile can choose a model per role (for example `gemini-3.8-flash` for a faster agent, `gemini-3.5-flash-lite` to sort heard sentences). The names are in [config/ia.toml](config/ia.toml).

<a id="où-tournent-les-modèles-open-source"></a>
## Where open source models run

- **Inference server `inference/`** (Python, single process): to be launched on the GX10 or on the pilot Mac. It exposes simple endpoints, each optional depending on what is installed: `/v1/chat/completions` is delegated to Ollama or vLLM (OpenAI-compatible API), and the server adds `/v1/audio/transcriptions` (whisper.cpp or faster-whisper), `/v1/audio/speech` (Piper), `/v1/detection`, `/v1/profondeur`, `/v1/pointage`, `/v1/empreintes`, and `/v1/sante` (what is available).
- **On the Pi**, only the small models that run in real time without hindering driving: speech detection, wake word, speech synthesis, low-cadence object detection.
- The rover keeps the same code: only the profile changes the model addresses and names.

<a id="état"></a>
## Status

| Role | Gemini | Open source |
|---|---|---|
| agent, vision, transcription | connected and verified with the real key | GX10: served and verified (Qwen3.6, Qwen3-ASR) |
| embodied pointing | Robotics ER 2 (vision role) | GX10: RoboBrain 2.5, verified |
| synthesis, detection, depth, segmentation, memory | — | GX10: served, connected, verified in simulation (21/10/2026) |
| speech, wake-up | built-in detector | to be written |

**Watchdog agent** (`ia.veille`): every 8 s, the vision model looks at the camera with the rover's measurements and summarizes the situation; the conversation agent uses this to respond, and an alert ("attention", "danger") is spoken aloud. Free and active by default on the GX10; with Gemini, only if `ia.vigilance` is enabled (one paid request per image).

**Qwen calls one function at a time**: with `appels_successifs = true` (gx10 profile), the agent retries the model until the instruction is complete ("move forward then turn": 12 on 12 vs 5 on 10 without retry, ≈ 1,9 s per instruction).

Mac disk: 2,4 Go free on 19 October 2026: no local models are downloaded.
