from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""STL des pièces imprimées de la remorque, orientées pour l'impression (posées à Z = 0, centrées),
STEP + F3D du document « remorque fieldmind », manifeste. La chape (côté rover) est
exportée par 00_chape_rover.py.

Orientations (aucun support) :
- rebords : retournés, la lèvre contre le plateau (le dessus du rebord sort lisse) ;
- supports de roue : couchés sur la face extérieure de la jambe ;
- boîtiers des servos : couchés sur leur face extérieure ; blocs arrière : debout sur leur face arrière ;
- ferrures des volets : debout, noeud vertical (bossages des inserts à l'horizontale) ;
- timon : à plat, dessous contre le plateau ; chape d'attelage : couchée sur son flanc ;
- bagues, entretoises, rondelles : à plat (bague : lèvre contre le plateau).
Les jantes, pneus et flasques sont ceux des roues avant : mêmes STL que la direction V04.
"""
import adsk.core, adsk.fusion, json, math, os
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/remorque/scripts/rm_helpers.py'), encoding='utf-8').read(), 'rm_helpers.py', 'exec'))
OUT = DOSSIER + '/exports'
PLATEAU = 180.0
V04_STL = '../../mounts/prototype/direction_v04_servo_traversant/exports/stl/'
# corps -> (fichier, quantité, couleur, rotation (axe, degrés), orientation)
PIECES = {
    'timon': ('timon', 1, 'bleu', None, 'à plat, dessous contre le plateau'),
    'support_roue_G': ('support_roue_G', 1, 'blanc', ('y', 90), 'couché sur la face extérieure de la jambe'),
    'support_roue_D': ('support_roue_D', 1, 'blanc', ('y', -90), 'couché sur la face extérieure de la jambe'),
    'bague_625_int_G': ('bague_625_dans_logement_608', 4, 'blanc', ('y', 90), 'lèvre contre le plateau'),
    'entretoise_625_G': ('entretoise_625_6.2mm', 2, 'blanc', ('y', 90), 'à plat'),
    'rondelle_tete_G': ('rondelle_tete_M5_2mm', 2, 'blanc', ('y', 90), 'à plat'),
    'cale_jambe_G': ('cale_jambe_3mm', 2, 'blanc', ('y', 90), 'à plat'),
}
for _n in ('AV_G', 'AR_G', 'AR_D', 'AV_D'):
    PIECES['rebord_' + _n] = ('rebord_' + _n, 1, 'bleu', ('x', 180), 'retourné, lèvre contre le plateau')
for _n, _r in (('M1_G', 90), ('M2_G', 90), ('M1_D', -90), ('M2_D', -90)):
    PIECES['joue_' + _n] = ('joue_' + _n, 1, 'bleu', ('y', _r), 'couchée sur sa face extérieure')
for _c, _r in (('G', 90), ('D', -90)):
    PIECES['boitier_servo_' + _c] = ('boitier_servo_' + _c, 1, 'blanc', ('y', _r), 'couché sur sa face extérieure (boîtier et butée)')
    PIECES['charniere_AR_' + _c] = ('charniere_AR_' + _c, 1, 'blanc', ('x', -90), 'debout sur sa face arrière')
    PIECES['ferrure_AV_' + _c] = ('ferrure_AV_' + _c, 1, 'blanc', ('x', 90), 'debout sur la face du palonnier')
    PIECES['ferrure_AR_' + _c] = ('ferrure_AR_' + _c, 1, 'blanc', ('x', -90), 'debout, noeud vertical')


def run(_context: str):
    d = design()                                      # document « remorque fieldmind »
    em = d.exportManager
    root = d.rootComponent
    os.makedirs(OUT + '/stl', exist_ok=True)
    man = {'pieces': [], 'plateau_mm': PLATEAU,
           'chape': 'exportée par 00_chape_rover.py (document du rover)',
           'roues_identiques_a_l_avant': {'quantite_par_stl': 2, 'stl': [V04_STL + f for f in ('jante_608ZZ.stl', 'pneu_TPU.stl', 'flasque_pneu.stl')],
                                          'note': 'mêmes fichiers que les roues avant de la direction V04 (2 de plus)'}}
    for o in root.occurrences:
        for b in o.component.bRepBodies:
            if b.name not in PIECES:
                continue
            nom, q, coul, rot, orient = PIECES[b.name]
            taille = stl(b, '%s/stl/%s.stl' % (OUT, nom), rot)
            man['pieces'].append({'stl': 'stl/%s.stl' % nom, 'quantite': q, 'couleur': coul, 'orientation': orient,
                                  'taille_impression_mm': taille, 'tient_sur_A1_mini': all(t <= PLATEAU for t in taille),
                                  'volume_cm3': round(b.volume, 1)})
    em.execute(em.createSTEPExportOptions(OUT + '/remorque.step', root))
    em.execute(em.createFusionArchiveExportOptions(OUT + '/remorque.f3d', root))
    open(OUT + '/manifest.json', 'w', encoding='utf-8').write(texte(json.dumps(man, ensure_ascii=False, indent=1)))
    print(json.dumps([(p['stl'], p['quantite'], p['taille_impression_mm'], p['tient_sur_A1_mini'], p['volume_cm3']) for p in man['pieces']], ensure_ascii=False))
