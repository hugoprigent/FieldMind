"""Client Pi → GX10 non bloquant pour le battement RC.

Une seule requête à la fois, identifiant d'image et horloge locale de capture.
Un résultat tardif ne devient jamais récent parce qu'il vient d'arriver.
"""

from __future__ import annotations

import json
import math
import time
from collections.abc import Callable

import aiohttp

from .modele import Cible


class VisionDistante:
    def __init__(
        self, url: str, jeton: str, calibration: str, horloge: Callable[[], float] = time.monotonic
    ) -> None:
        if not jeton or not url.startswith(("http://", "https://")):
            raise ValueError("URL et jeton de vision requis")
        self.url, self.jeton, self.calibration = url.rstrip("/"), jeton, calibration
        self.horloge = horloge
        self._occupee = False
        self._derniere_sequence = -1
        self.erreur: str | None = None

    async def estimer(self, jpeg: bytes, sequence: int, t_capture: float) -> Cible | None:
        if self._occupee or sequence <= self._derniere_sequence:
            return None
        if not math.isfinite(t_capture) or not 0 <= self.horloge() - t_capture <= 0.5:
            return None
        self._occupee = True
        self._derniere_sequence = sequence
        self.erreur = None
        try:
            async with (
                aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=0.3)) as s,
                s.post(
                    self.url + "/analyse",
                    data=jpeg,
                    headers={
                        "X-Fieldmind-Jeton": self.jeton,
                        "X-Image-Sequence": str(sequence),
                        "X-Calibration": self.calibration,
                        "Content-Type": "image/jpeg",
                    },
                ) as reponse,
            ):
                reponse.raise_for_status()
                if reponse.content_length is not None and reponse.content_length > 8192:
                    raise ValueError("réponse vision trop volumineuse")
                brut = await reponse.content.read(8193)
                if len(brut) > 8192:
                    raise ValueError("réponse vision trop volumineuse")
                d = json.loads(brut)
            if d.get("sequence") != sequence or d.get("calibration") != self.calibration:
                raise ValueError("réponse d'une autre image ou calibration")
            if not 0 <= self.horloge() - t_capture <= 0.5:
                raise ValueError("résultat de vision périmé")
            v = d.get("cible")
            if v is None:
                return None
            cible = Cible(
                t=t_capture,
                sequence=sequence,
                avant_m=float(v["avant_m"]),
                droite_m=float(v["droite_m"]),
                hauteur_m=float(v["hauteur_m"]),
                qualite=float(v["qualite"]),
                source="gx10_camera_bas",
            )
            if not cible.valide(self.horloge()):
                raise ValueError("pose distante non valide")
            return cible
        except (aiohttp.ClientError, TimeoutError, ValueError, TypeError, KeyError) as e:
            self.erreur = str(e)
            return None
        finally:
            self._occupee = False
