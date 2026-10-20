from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Supports des moteurs arrière agrandis, en place dans le document du rover : la pièce Onshape
(bloc de 40 mm, alésage Ø46 de 37,5 mm, 2 vis M8 en ligne) est remplacée dans le composant
D4_REF_Support_moteur_arriere (même pièce des deux côtés, la gauche tournée de 180°).

- Le moteur est tenu sur 85 mm au lieu de 40, dans un alésage étagé comme lui (cotes de Hugo) : Ø46,3
  sur le réducteur (55,5 mm), puis Ø42,3 sur le moteur. Épaulement Ø52 de 2,5 mm et 4 trous Ø4 côté roue,
  comme la pièce Onshape.
- Collier : fente de 2 mm sous le moteur, serrée par 2 vis M6×25 et 2 écrous M6 dans une quille
  sous l'alésage. Le moteur est serré quelle que soit la tolérance d'impression.
- Semelle de 95 × 108 × 10 mm sous la planche, 4 vis H M8 : les 2 trous existants (x ±130) et 2
  nouveaux à x ±68. Tête sous la semelle, rondelle et écrou sur la planche. Encoche Ø14 pour la vis
  M6 de la coque qui passe à côté (x ±90, y 432,6).
- Repère du composant : origine sur l'axe du moteur, au droit de la face côté roue ; u = X vers
  l'intérieur négatif, v = Y, w = Z (dessous de planche à w = 30).
Imprimer debout sur la face côté roue (alésage vertical, sans support).
Ajoute les 4 nouveaux trous à trous_composants.json (05_percages.py), exporte le STL.
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/04_exporter.py'), encoding='utf-8').read(), '04_exporter.py', 'exec'))
DOSSIER = str(_FM_ROOT / 'cao/prototype_final')
L = 85.0                          # longueur tenue le long du moteur
ALESAGE = [(46.3, 55.5)]                      # alésage étagé comme le moteur : Ø46,3 sur le réducteur (55,5 mm), puis Ø42,3
ALESAGE_FOND = 42.3
EPAUL_D, EPAUL_P = 52.0, 2.5
SEM_L, SEM_V, SEM_W0, SEM_EP = 95.0, 54.0, 20.0, 10.0   # semelle : 95 de long, ±54 en Y, de w 20 à 30
BLOC_V, BLOC_BAS = 30.0, -30.0                # bloc ±30 jusqu'à w -30 (z 70)
QUILLE_V, QUILLE_BAS = 12.0, -42.0            # quille du collier ±12 jusqu'à w -42 (z 58)
FENTE = 2.0
PINCE_U, PINCE_W, PINCE_D, ECROU_M6 = (-18.0, -67.0), -36.0, 6.5, 10.2
M8_TROUS = [(-20.0, 45.0), (-20.0, -45.0), (-82.0, 45.0), (-82.0, -45.0)]   # x ±68 : écrous entre les contrôleurs BLDC et le bossage M6 de la coque
ENCOCHE = (-60.0, 52.6, 14.0)                 # vis M6 de la coque (K) à côté de la semelle, des deux côtés
X_ROUE, Y_AXE = 150.0, 380.0


def ysym(c, nom, pts_uw, v0, total, op, bodies):
    """Profil dans le plan v = v0 (points u, w), extrudé symétriquement sur `total` mm."""
    s = sketch(c, plane_y(c, v0, nom), nom)
    poly3(s, [(u, v0, w) for u, w in pts_uw])
    i = c.features.extrudeFeatures.createInput(profiles(s), op)
    i.setSymmetricExtent(V(total), True)
    if bodies is not None:
        i.participantBodies = list(bodies)
    f = c.features.extrudeFeatures.add(i)
    f.name = 'extr_' + nom
    return f


def cercle_uw(u, w, d, n=32):
    return [(u + d / 2 * math.cos(2 * math.pi * k / n), w + d / 2 * math.sin(2 * math.pi * k / n)) for k in range(n)]


def support_moteur(c, ep=None):
    """Construit le support dans le composant c (repère du support) et renvoie son corps ; avec ep
    (épaisseur de planche), ajoute aussi la visserie de référence."""
    ch = 6.0
    sem = [(-SEM_L + ch, -SEM_V), (-ch, -SEM_V), (0, -SEM_V + ch), (0, SEM_V - ch), (-ch, SEM_V), (-SEM_L + ch, SEM_V), (-SEM_L, SEM_V - ch), (-SEM_L, -SEM_V + ch)]
    b = body(zpoly(c, 'semelle', sem, SEM_W0, SEM_EP), 'support_moteur_arriere', BLANC)
    prof = [(-BLOC_V, SEM_W0 + 1), (BLOC_V, SEM_W0 + 1), (BLOC_V, BLOC_BAS + 4), (BLOC_V - 4, BLOC_BAS), (QUILLE_V, BLOC_BAS),
            (QUILLE_V, QUILLE_BAS), (-QUILLE_V, QUILLE_BAS), (-QUILLE_V, BLOC_BAS), (-BLOC_V + 4, BLOC_BAS), (-BLOC_V, BLOC_BAS + 4)]
    xpoly(c, 'bloc', 0, prof, -L, JOIN, [b])
    xcyl(c, 'alesage', 1, 0, 0, ALESAGE_FOND, -(SEM_L + 2), CUT, [b])   # aussi à travers le bout de semelle qui dépasse du bloc
    for k, (da, la) in enumerate(ALESAGE):
        xcyl(c, 'alesage_%d' % k, 1, 0, 0, da, -(la + 1), CUT, [b])
    xcyl(c, 'epaulement', 1, 0, 0, EPAUL_D, -(EPAUL_P + 1), CUT, [b])
    for k, (sv, sw) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        xcyl(c, 'trou_face_%d' % k, 1, sv * 23, sw * 23, 4.0, -11, CUT, [b])
    zbox(c, 'fente', -L - 1, -FENTE / 2, 1, FENTE / 2, QUILLE_BAS - 1, -20 - QUILLE_BAS + 1, CUT, [b])
    for k, u in enumerate(PINCE_U):
        ysym(c, 'pince_%d' % k, cercle_uw(u, PINCE_W, PINCE_D), 0, 2 * QUILLE_V + 4, CUT, [b])
        r = ECROU_M6 / math.sqrt(3)
        hexa = [(u + r * math.cos(math.pi / 2 + j * math.pi / 3 + math.pi / 2), PINCE_W + r * math.sin(math.pi / 2 + j * math.pi / 3 + math.pi / 2)) for j in range(6)]
        ysym(c, 'ecrou_pince_%d' % k, hexa, -QUILLE_V, 11.0, CUT, [b])
    for k, (u, v) in enumerate(M8_TROUS):
        zcyl(c, 'trou_M8_%d' % k, u, v, 9.0, SEM_W0 - 1, SEM_EP + 2, CUT, [b])
    for k, sv in enumerate((1, -1)):
        zcyl(c, 'encoche_K_%d' % k, ENCOCHE[0], sv * ENCOCHE[1], ENCOCHE[2], SEM_W0 - 1, SEM_EP + 2, CUT, [b])
    # visserie de référence : vis H M8 (tête sous la semelle), rondelle et écrou sur la planche ; vis M6 du collier
    if ep is None:
        return b
    for k, (u, v) in enumerate(M8_TROUS):
        t = body(zpoly(c, 'REF_tete_M8_%d' % k, hexagon(None, u, v, 13.0), SEM_W0 - 5.3, 5.3), 'REF_vis_H_M8_support_moteur_%d' % k, METAL)
        zcyl(c, 'REF_tige_M8_%d' % k, u, v, 8.0, SEM_W0, SEM_EP + ep + 1.6 + 6.5 + 1, JOIN, [t])
        body(zcyl(c, 'REF_rondelle_M8_%d' % k, u, v, 16.0, 30 + ep, 1.6), 'REF_rondelle_M8_support_moteur_%d' % k, METAL)
        body(zpoly(c, 'REF_ecrou_M8_%d' % k, hexagon(None, u, v, 13.0), 30 + ep + 1.6, 6.5), 'REF_ecrou_M8_support_moteur_%d' % k, METAL)
    for k, u in enumerate(PINCE_U):
        body(ysym(c, 'REF_vis_M6_pince_%d' % k, cercle_uw(u, PINCE_W, 6.0), 0, 2 * QUILLE_V + 2, NEW, None), 'REF_vis_M6x25_collier_%d' % k, METAL)
    return b


def run(_context: str):
    d = design()
    v04 = next(o for o in d.rootComponent.occurrences if o.component.name == 'FM_Direction_V04')
    c = next(o for o in v04.childOccurrences if o.component.name == 'D4_REF_Support_moteur_arriere').component
    for coll in (c.features.baseFeatures, c.features.extrudeFeatures, c.sketches, c.constructionPlanes):
        for x in reversed([coll.item(i) for i in range(coll.count)]):
            try:
                if x.isValid:
                    x.deleteMe()
            except RuntimeError:
                pass
    for b in list(c.bRepBodies):
        if b.isValid:
            b.deleteMe()
    b = support_moteur(c, ev('d4_planche_ep'))
    # trous de la planche : les 2 nouveaux par support (les anciens, x ±130, sont déjà dans la planche V04)
    tc = json.load(open(DOSSIER + '/trous_composants.json', encoding='utf-8'))
    tc['trous'] = [t for t in tc['trous'] if t[3] != 'support_moteur']
    for s in (1, -1):
        for u, v in M8_TROUS:
            if u < -50:
                tc['trous'].append([round(s * (X_ROUE + u), 2), round(Y_AXE + s * v, 2), 8.5, 'support_moteur'])
    json.dump(tc, open(DOSSIER + '/trous_composants.json', 'w', encoding='utf-8'), indent=1)
    taille = stl(b, DOSSIER + '/exports/stl/support_moteur_arriere.stl', ('y', 90))
    bx = b.boundingBox
    print(json.dumps({'taille_impression_mm': taille, 'volume_cm3': round(b.volume, 1), 'morceaux': b.lumps.count,
                      'bbox_local': [round(v * 10, 1) for v in bx.minPoint.asArray() + bx.maxPoint.asArray()],
                      'trous_planche': [t for t in tc['trous'] if t[3] == 'support_moteur']}))
