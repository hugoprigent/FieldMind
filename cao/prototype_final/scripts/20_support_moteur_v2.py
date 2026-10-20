from pathlib import Path as _FM_Path
import os as _FM_os
_FM_ROOT = _FM_Path(_FM_os.environ.get("FIELDMIND_ROOT", _FM_Path.cwd())).resolve()

"""Support moteur arrière V2, plus léger, dans le document « supports roues fieldmind », à côté du V1.

Mêmes interfaces que le V1 (16_supports_moteurs.py) : alésage étagé Ø46,3 / Ø42,3, épaulement Ø52,
4 trous Ø4 côté roue, collier fendu serré par 2 vis M6×25, semelle de 95 × 108 à 4 vis H M8 aux
mêmes trous, encoche de la vis M6 de la coque. Seule la matière qui ne porte pas est retirée :
- **tube rond** de 5,85 mm de paroi autour du moteur (R 29 sur le réducteur, R 27 sur le moteur) au
  lieu du bloc carré, avec une **bride carrée de 12 mm** côté roue seulement (elle garde les 4 trous
  Ø4 et l'épaulement) ;
- **collier à deux oreilles** de 18 mm autour des vis M6 au lieu d'une quille sur toute la longueur
  (le tube reste fendu et serré sur toute sa longueur) ;
- **semelle évidée par-dessous** : poches de 6 mm qui laissent une peau de 4 mm contre la
  planche, une nervure transversale au milieu, un rail plein de 10 mm le long du tube et des
  bossages pleins Ø18 autour des vis M8 (la tête de vis appuie sur 10 mm comme au V1).
Le V2 tient dans l'enveloppe du V1 : aucune collision nouvelle.

Dans l'assemblage, les deux versions sont présentes : V1 à gauche (+X), V2 à droite (−X) pour les
comparer ; les deux autres exemplaires (V1 droite, V2 gauche) existent mais sont masqués. Le document
des supports est enregistré, la référence du rover est mise à jour (le rover n'est pas enregistré).
Exporte le STL du V2 (même orientation d'impression : debout sur la face côté roue).
"""
import adsk.core, adsk.fusion, json, math
exec(compile(open(str(_FM_ROOT / 'cao/prototype_final/scripts/16_supports_moteurs.py'), encoding='utf-8').read(), '16_supports_moteurs.py', 'exec'))
DOC_ROVER, DOC_SUPPORTS = 'prtotype final fieldmind', 'supports roues fieldmind'
NOM_V1, NOM_V2 = 'SR_IMP_Support_moteur_arriere', 'SR_IMP_Support_moteur_arriere_V2'
R_TUBE = 29.0                  # paroi de 5,85 mm autour de l'alésage Ø46,3 (réducteur)
R_TUBE_MOTEUR = 27.0           # paroi de 5,85 mm autour de l'alésage Ø42,3 (moteur)
OREILLE = 18.0                 # longueur de chaque oreille du collier, centrée sur sa vis M6
BRIDE = 12.0                   # bride carrée côté roue (trous Ø4 à 32,5 mm de l'axe, épaulement)
POCHE = 6.0                    # profondeur des poches sous la semelle (peau de 4 mm)
BOSSAGE = 18.0                 # bossages pleins autour des vis M8
PETG = 1.27                    # g/cm³


def design():
    """Design du document actif (les helpers du rover exigent sinon le document du rover)."""
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


def matrice(tx, ty, tz, demi_tour=False):
    m = adsk.core.Matrix3D.create()
    if demi_tour:
        m.setToRotation(math.pi, adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    m.translation = adsk.core.Vector3D.create(tx / 10, ty / 10, tz / 10)
    return m


def profil_tube(r, n=48):
    """Profil (v, w) du tube de rayon r : haut noyé dans la semelle, puis tout le bas en arc."""
    w0 = SEM_W0
    a0 = math.atan2(w0, math.sqrt(r ** 2 - w0 ** 2))          # l'arc rejoint le dessous de la semelle
    v_haut = math.sqrt(r ** 2 - w0 ** 2) + 0.5
    pts = [(-v_haut, w0 + 1), (v_haut, w0 + 1)]
    a1 = math.pi - a0 - 2 * math.pi                            # même point à gauche, en passant par le bas
    pts += [(r * math.cos(a0 + (a1 - a0) * k / n), r * math.sin(a0 + (a1 - a0) * k / n)) for k in range(n + 1)]
    return pts


def support_moteur_v2(c, ep=None):
    ch = 6.0
    sem = [(-SEM_L + ch, -SEM_V), (-ch, -SEM_V), (0, -SEM_V + ch), (0, SEM_V - ch), (-ch, SEM_V), (-SEM_L + ch, SEM_V), (-SEM_L, SEM_V - ch), (-SEM_L, -SEM_V + ch)]
    b = body(zpoly(c, 'v2_semelle', sem, SEM_W0, SEM_EP), 'support_moteur_arriere_v2', BLANC)
    # poches sous la semelle, de part et d'autre du rail central, nervure de 5 mm au milieu
    for k, sv in enumerate((1, -1)):
        v0, v1 = sorted((sv * 31.0, sv * 49.0))
        for j, (u_ext, u_int) in enumerate(((-5.0, -48.5), (-53.5, -90.0))):
            zbox(c, 'v2_poche_%d_%d' % (k, j), u_int, v0, u_ext, v1, SEM_W0 - 1, POCHE + 1, CUT, [b])
    for k, (u, v) in enumerate(M8_TROUS):
        zcyl(c, 'v2_bossage_M8_%d' % k, u, v, BOSSAGE, SEM_W0, POCHE + 0.5, JOIN, [b])
    # bride carrée côté roue (profil du V1) puis tube rond jusqu'au bout
    carre = [(-BLOC_V, SEM_W0 + 1), (BLOC_V, SEM_W0 + 1), (BLOC_V, BLOC_BAS + 4), (BLOC_V - 4, BLOC_BAS), (QUILLE_V, BLOC_BAS),
             (QUILLE_V, QUILLE_BAS), (-QUILLE_V, QUILLE_BAS), (-QUILLE_V, BLOC_BAS), (-BLOC_V + 4, BLOC_BAS), (-BLOC_V, BLOC_BAS + 4)]
    xpoly(c, 'v2_bride', 0, carre, -BRIDE, JOIN, [b])
    l_reducteur = ALESAGE[0][1] + 1.0                          # tube R 29 sur le réducteur, R 27 ensuite
    xpoly(c, 'v2_tube_reducteur', -(BRIDE - 1), profil_tube(R_TUBE), -(l_reducteur - BRIDE + 1), JOIN, [b])
    xpoly(c, 'v2_tube_moteur', -(l_reducteur - 1), profil_tube(R_TUBE_MOTEUR), -(L - l_reducteur + 1), JOIN, [b])
    oreille = [(QUILLE_V, -18.0), (QUILLE_V, QUILLE_BAS), (-QUILLE_V, QUILLE_BAS), (-QUILLE_V, -18.0)]
    for k, u in enumerate(PINCE_U):                            # oreilles du collier autour des vis M6
        u0 = min(-0.0, u + OREILLE / 2)
        xpoly(c, 'v2_oreille_%d' % k, u0, oreille, -OREILLE, JOIN, [b])
    # mêmes usinages que le V1
    xcyl(c, 'v2_alesage', 1, 0, 0, ALESAGE_FOND, -(SEM_L + 2), CUT, [b])
    for k, (da, la) in enumerate(ALESAGE):
        xcyl(c, 'v2_alesage_%d' % k, 1, 0, 0, da, -(la + 1), CUT, [b])
    xcyl(c, 'v2_epaulement', 1, 0, 0, EPAUL_D, -(EPAUL_P + 1), CUT, [b])
    for k, (sv, sw) in enumerate(((1, 1), (1, -1), (-1, 1), (-1, -1))):
        xcyl(c, 'v2_trou_face_%d' % k, 1, sv * 23, sw * 23, 4.0, -11, CUT, [b])
    zbox(c, 'v2_fente', -L - 1, -FENTE / 2, 1, FENTE / 2, QUILLE_BAS - 1, -20 - QUILLE_BAS + 1, CUT, [b])
    for k, u in enumerate(PINCE_U):
        ysym(c, 'v2_pince_%d' % k, cercle_uw(u, PINCE_W, PINCE_D), 0, 2 * QUILLE_V + 4, CUT, [b])
        r = ECROU_M6 / math.sqrt(3)
        hexa = [(u + r * math.cos(math.pi / 2 + j * math.pi / 3 + math.pi / 2), PINCE_W + r * math.sin(math.pi / 2 + j * math.pi / 3 + math.pi / 2)) for j in range(6)]
        ysym(c, 'v2_ecrou_pince_%d' % k, hexa, -QUILLE_V, 11.0, CUT, [b])
    for k, (u, v) in enumerate(M8_TROUS):
        zcyl(c, 'v2_trou_M8_%d' % k, u, v, 9.0, SEM_W0 - 1, SEM_EP + 2, CUT, [b])
    for k, sv in enumerate((1, -1)):
        zcyl(c, 'v2_encoche_K_%d' % k, ENCOCHE[0], sv * ENCOCHE[1], ENCOCHE[2], SEM_W0 - 1, SEM_EP + 2, CUT, [b])
    if ep is None:
        return b
    for k, (u, v) in enumerate(M8_TROUS):          # visserie de référence, identique au V1
        t = body(zpoly(c, 'REF_tete_M8_%d' % k, hexagon(None, u, v, 13.0), SEM_W0 - 5.3, 5.3), 'REF_vis_H_M8_support_moteur_%d' % k, METAL)
        zcyl(c, 'REF_tige_M8_%d' % k, u, v, 8.0, SEM_W0, SEM_EP + ep + 1.6 + 6.5 + 1, JOIN, [t])
        body(zcyl(c, 'REF_rondelle_M8_%d' % k, u, v, 16.0, 30 + ep, 1.6), 'REF_rondelle_M8_support_moteur_%d' % k, METAL)
        body(zpoly(c, 'REF_ecrou_M8_%d' % k, hexagon(None, u, v, 13.0), 30 + ep + 1.6, 6.5), 'REF_ecrou_M8_support_moteur_%d' % k, METAL)
    for k, u in enumerate(PINCE_U):
        body(ysym(c, 'REF_vis_M6_pince_%d' % k, cercle_uw(u, PINCE_W, 6.0), 0, 2 * QUILLE_V + 2, NEW, None), 'REF_vis_M6x25_collier_%d' % k, METAL)
    return b


def run(_context: str):
    a = app()
    rover = next(d_ for d_ in a.documents if d_.name.startswith(DOC_ROVER))
    candidats = [d_ for d_ in a.documents if d_.name.startswith(DOC_SUPPORTS) and d_.dataFile is not None]
    doc = max(candidats, key=lambda d_: d_.dataFile.versionNumber)
    doc.activate()
    d = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    root = d.rootComponent
    for o in [o for o in root.occurrences if o.component.name.startswith(NOM_V2)]:
        o.deleteMe()
    v1 = [o for o in root.occurrences if o.component.name.startswith(NOM_V1) and not o.component.name.startswith(NOM_V2)]
    om = root.occurrences.addNewComponent(matrice(X_ROUE, Y_AXE, 100.0))
    om.isGroundToParent = False
    cm = om.component
    cm.name = NOM_V2
    b2 = support_moteur_v2(cm, 5.0)
    o_droite = root.occurrences.addExistingComponent(cm, matrice(-X_ROUE, Y_AXE, 100.0, True))
    # V1 à gauche (+X), V2 à droite (−X) ; les deux autres exemplaires masqués
    for o in v1:
        o.isLightBulbOn = o.transform2.translation.x > 0
    om.isLightBulbOn = False
    o_droite.isLightBulbOn = True
    b1 = next(b for b in v1[0].component.bRepBodies if b.name == 'support_moteur_arriere')
    sortie = {'stl_mm': stl(b2, DOSSIER + '/exports/stl/support_moteur_arriere_v2.stl', ('y', 90))}
    v1_cm3, v2_cm3 = b1.volume, b2.volume
    bilan = {'v1_cm3': round(v1_cm3, 1), 'v2_cm3': round(v2_cm3, 1),
             'v1_g_plein': round(v1_cm3 * PETG), 'v2_g_plein': round(v2_cm3 * PETG),
             'gain_pct': round(100 * (1 - v2_cm3 / v1_cm3), 1), 'morceaux_v2': b2.lumps.count,
             'surface_v1_cm2': round(b1.area, 1), 'surface_v2_cm2': round(b2.area, 1)}
    json.dump(bilan, open(DOSSIER + '/support_moteur_v2_bilan.json', 'w', encoding='utf-8'), indent=1)
    doc.save('Support moteur V2 plus leger, V1 conserve (20_support_moteur_v2.py)')
    # rover : référence mise à jour (sans enregistrer le document du rover)
    rover.activate()
    maj = False
    for r in rover.documentReferences:
        if r.dataFile and r.dataFile.name == DOC_SUPPORTS:
            try:
                r.getLatestVersion()
                maj = True
            except RuntimeError as e:
                bilan['reference_erreur'] = str(e)
    rover.activate()
    bilan.update({'document': doc.name, 'version': doc.dataFile.versionNumber, 'reference_maj': maj, **sortie})
    print(json.dumps(bilan, ensure_ascii=False))
