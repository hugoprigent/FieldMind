# Modèles d'IA du rover : Gemini maintenant, open source ensuite

Chaque fonction d'IA est un **rôle** avec une interface fixe. Le profil choisi dans le poste
(Réglages > IA) dit qui la sert : l'API Gemini aujourd'hui, des modèles open source demain (GX10,
le Mac de l'opérateur, ou le Pi pour les plus petits). Le reste du rover ne voit que le rôle.

**Aucun rôle n'est dans la boucle de sécurité.** Un modèle peut ajouter de la prudence
(ralentir devant une personne détectée), jamais en retirer ; s'il manque ou tombe, le rover se
conduit, évite les obstacles et s'arrête exactement comme sans lui.

## Correspondance des rôles

| Rôle | Ce qu'il fait | Gemini (profil `gemini`) | GX10 (`gx10`) | Mac M3 16 Go (`mac`) | Pi 4 (`pi`) |
|---|---|---|---|---|---|
| `agent` | consigne → actions (appel de fonctions) | `gemini-4-argon` | Qwen3.6-35B-A3B NVFP4 (vLLM) | Gemma 4 E4B ou Qwen3 8B, 4 bits (Ollama) | — |
| `vision` | décrire, pointer un objet (`[y, x]` 0–1000), boîtes | `gemini-robotics-er-2-preview` | Qwen3.6-35B-A3B (même serveur, pointage `[x, y]`) | Moondream 2 / Qwen2.5-VL 3B (Ollama) | — |
| `pointage` | modèle incarné : pointer un objet, une zone (`chercher`) | (vision) | RoboBrain 2.5-4B (BAAI) | — | — |
| `transcription` | parole → texte | `gemini-3.5-transcribe` | Qwen3-ASR-1.7B (ou Whisper large-v3-turbo) | Whisper small (whisper.cpp, Metal) | Whisper base (whisper.cpp), lent |
| `synthese` | texte → voix du rover | `gemini-3.8-flash-tts` | Qwen3-TTS-0.6B | Piper | Piper |
| `detection` | objets en continu (personnes, animaux, véhicules) | ER 2 à faible cadence (0,2 Hz) | D-FINE xlarge (0,4 Hz) | YOLO nano/small (ONNX) | YOLO nano 320 px, ~3 i/s |
| `profondeur` | profondeur d'une seule caméra (complète la stéréo) ; segmentation SAM 2.1 | — | Depth Anything V2 métrique + SAM 2.1 | Depth Anything V2 Small (ONNX) | — |
| `parole` | détecter la parole dans le micro | détecteur intégré | Silero VAD | Silero VAD | Silero VAD (< 1 ms par bloc) |
| `eveil` | mot d'éveil (« rover ») | — | openWakeWord | openWakeWord | openWakeWord |
| `memoire` | empreintes d'images (reconnaître un lieu, « un objet comme ça ») | `gemini-embedding-2-preview` | Qwen3-VL-Embedding-2B | SigLIP 2 base (ONNX) | — |

En ligne, tous les modèles de l'API Gemini sont utilisables : le profil peut choisir un modèle
par rôle (par exemple `gemini-3.8-flash` pour un agent plus rapide, `gemini-3.5-flash-lite`
pour trier les phrases entendues). Les noms sont dans [config/ia.toml](config/ia.toml).

## Où tournent les modèles open source

- **Serveur d'inférence `inference/`** (Python, un seul processus) : à lancer sur le GX10 ou sur
  le Mac qui pilote. Il expose des points d'accès simples, chacun facultatif selon ce qui est
  installé : `/v1/chat/completions` est délégué à Ollama ou vLLM (API compatible OpenAI), et
  le serveur ajoute `/v1/audio/transcriptions` (whisper.cpp ou faster-whisper),
  `/v1/audio/speech` (Piper), `/v1/detection`, `/v1/profondeur`, `/v1/pointage`,
  `/v1/empreintes`, et `/v1/sante` (ce qui est disponible).
- **Sur le Pi**, seulement les petits modèles qui tiennent le temps réel sans gêner la conduite :
  détection de parole, mot d'éveil, synthèse vocale, détection d'objets à basse cadence.
- Le rover garde le même code : seul le profil change l'adresse et le nom des modèles.

## État

| Rôle | Gemini | Open source |
|---|---|---|
| agent, vision, transcription | branchés, testés avec un faux serveur, pas encore avec une vraie clé | profils `gx10` (vLLM) prêts ; `mac` à écrire |
| synthèse, détection, profondeur, parole, éveil, mémoire | à écrire | à écrire |

Disque du Mac : 2,4 Go libres au 19 octobre 2026 : aucun modèle local n'est téléchargé.
