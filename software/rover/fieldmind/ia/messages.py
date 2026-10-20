"""Messages publiés par l'IA sur le bus, au format du poste (PROTOCOLE.md : `chat`, `voix`).

Ils portent déjà leur `type` : le serveur web peut les relayer tels quels. `t` = heure murale
(comme les messages `journal`), les identifiants sont « m12 » (chat) et « v3 » (vision).
"""

from __future__ import annotations

import itertools
import time
from typing import Any

from fieldmind.bus import Bus, S
from fieldmind.types import Action

_COMPTEURS = {"m": itertools.count(1), "v": itertools.count(1)}


def nouvel_id(prefixe: str) -> str:
    return f"{prefixe}{next(_COMPTEURS.setdefault(prefixe, itertools.count(1)))}"


def publier_chat(bus: Bus, role: str, texte: str, origine: str = "texte",
                 plan: dict[str, Any] | None = None) -> dict[str, Any]:
    """`role` : operateur | rover | systeme ; `plan` : {"id": "p7", "actions": [...]}."""
    message: dict[str, Any] = {"type": "chat", "id": nouvel_id("m"), "t": time.time(),
                               "role": role, "texte": texte, "origine": origine}
    if plan is not None:
        message["plan"] = plan
    bus.publier(S.CHAT, message)
    return message


def plan_pour_chat(plan_id: str | None, actions: list[Action]) -> dict[str, Any]:
    return {"id": plan_id, "actions": actions}


def publier_voix(bus: Bus, evenement: str, **champs: Any) -> dict[str, Any]:
    """`evenement` : debut_parole | fin_parole | transcription | ignore | erreur."""
    message = {"type": "voix", "evenement": evenement, "t": time.time(), **champs}
    bus.publier(S.VOIX, message)
    return message
