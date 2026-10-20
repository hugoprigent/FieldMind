"""Budgets de temps des tests : vérifiés seulement quand la machine n'est pas surchargée.

Les temps mesurés (stéréo, rendu, planificateur…) n'ont de sens que sur une machine calme ;
avec d'autres programmes lourds en parallèle (navigateurs, simulateurs, CAO), un dépassement
ne dit rien du code. `dans_le_budget(duree, budget)` est vrai si la durée tient le budget, ou si
la charge moyenne dépasse le nombre de cœurs (le test signale alors la charge sans échouer).
"""

from __future__ import annotations

import os
import warnings


def machine_surchargee() -> bool:
    try:
        return os.getloadavg()[0] > (os.cpu_count() or 4)
    except OSError:
        return False


def dans_le_budget(duree: float, budget: float) -> bool:
    if duree < budget:
        return True
    if machine_surchargee():
        warnings.warn(f"budget de temps non vérifié (machine chargée) : {duree * 1000:.0f} ms "
                      f"pour {budget * 1000:.0f} ms", stacklevel=2)
        return True
    return False
