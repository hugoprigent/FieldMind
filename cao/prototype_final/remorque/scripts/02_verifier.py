from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Contrôles de la remorque seule, dans son document « remorque fieldmind » :
- collisions entre toutes ses pièces (pneu/jante emmanchés exclus), volets dépliés puis repliés ;
- jeux : pneus contre rebords, supports et volets (dépliés et repliés), jantes contre supports ;
- pièces d'un seul tenant, corps orphelins, tailles.
Laisse les volets REPLIÉS (position de route) : c'est l'état enregistré et vu dans l'assemblage du rover.
L'attelage et le braquage se contrôlent dans le document du rover (05_verifier_attelage.py).
Écrit verification_remorque.json.
"""
import adsk.core, adsk.fusion, json, math, re
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))


def collisions(corps):
    col = adsk.core.ObjectCollection.create()
    for b in corps:
        col.add(b)
    i = design().createInterferenceInput(col)
    i.areCoincidentFacesIncluded = False
    out = []
    for r in design().analyzeInterference(i):
        a, b = r.entityOne, r.entityTwo
        if {a.name.split('_')[0], b.name.split('_')[0]} == {'pneu', 'jante'}:
            continue
        out.append({'a': a.name, 'b': b.name, 'volume_mm3': round(r.interferenceBody.volume * 1000, 3)})
    return out


def volets(replies):
    volets_replies(replies)                            # par les liaisons pivot des charnières


def run(_context: str):
    root = design().rootComponent
    corps = []
    for o in root.occurrences:
        corps += list(o.bRepBodies)
        for oo in o.childOccurrences:
            corps += list(oo.bRepBodies)
    par_nom = lambda n: next(b for b in corps if b.name == n)
    mm = app().measureManager
    jeu = lambda a, bs: round(min(mm.measureMinimumDistance(a, b).value for b in bs) * 10, 2)
    res = {'jeux_mm': {}}
    for etat in (False, True):
        volets(etat)
        res['collisions_volets_' + ('replies' if etat else 'deplies')] = collisions(corps)
        for cote in ('G', 'D'):
            vol = [b for b in corps if b.name in ('REF_volet_contreplaque_' + cote, 'ferrure_AV_' + cote, 'ferrure_AR_' + cote)]
            res['jeux_mm']['pneu_%s/volet_%s' % (cote, 'replie' if etat else 'deplie')] = jeu(par_nom('pneu_TPU_remorque_' + cote), vol)
            res['jeux_mm']['volet_%s_%s/boitier_et_bloc_arriere' % (cote, 'replie' if etat else 'deplie')] = jeu(
                par_nom('REF_volet_contreplaque_' + cote), [par_nom('boitier_servo_' + cote), par_nom('charniere_AR_' + cote)])
    for cote in ('G', 'D'):
        pneu = par_nom('pneu_TPU_remorque_' + cote)
        autour = [b for b in corps if b.name.startswith(('rebord_', 'joue_', 'REF_planche', 'support_roue', 'timon', 'console', 'charniere', 'boitier'))]
        res['jeux_mm']['pneu_%s/rebords_planche_supports' % cote] = jeu(pneu, autour)
        res['jeux_mm']['jante_%s/support' % cote] = jeu(par_nom('jante_remorque_' + cote), [par_nom('support_roue_' + cote)])
        # boîtier du servo dans le coin coupé : rebord avant et planche autour, servo dans son logement
        boit = par_nom('boitier_servo_' + cote)
        res['jeux_mm']['boitier_%s/rebord_AV' % cote] = jeu(boit, [par_nom('rebord_AV_' + cote)])
        sg = [b for b in corps if b.name.startswith('REF_SG90_' + cote)]
        res['jeux_mm']['SG90_%s/planche_et_rebord_AV' % cote] = jeu(par_nom('REF_planche_remorque'), sg) if sg else None
        res['jeux_mm']['SG90_%s/rebord_AV' % cote] = jeu(par_nom('rebord_AV_' + cote), sg) if sg else None
    res['pieces'], res['morceaux'] = {}, {}
    for b in corps:
        if b.name.startswith('REF_'):
            continue
        bx = b.boundingBox
        res['pieces'][b.name] = sorted(round((bx.maxPoint.asArray()[i] - bx.minPoint.asArray()[i]) * 10, 1) for i in range(3))
        res['morceaux'][b.name] = b.lumps.count
    res['pieces_en_plusieurs_morceaux'] = [k for k, v in res['morceaux'].items() if v != 1]
    res['corps_orphelins'] = [b.name for b in corps if re.fullmatch(r'Corps\d+|Body\d+', b.name)]
    volets(True)                                       # état enregistré : volets repliés
    json.dump(res, open(DOSSIER + '/verification_remorque.json', 'w', encoding='utf-8'), indent=1, ensure_ascii=False)
    print(json.dumps({k: res[k] for k in ('collisions_volets_deplies', 'collisions_volets_replies', 'jeux_mm',
                                          'pieces_en_plusieurs_morceaux', 'corps_orphelins')}, ensure_ascii=False)[:5000])
