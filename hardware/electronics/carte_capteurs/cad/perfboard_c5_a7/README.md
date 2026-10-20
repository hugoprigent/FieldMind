# Projet courant A7 — une perfboard 4 × 6 cm, tous les fils dessous

Ouvrir `fieldmind_c5_a7.kicad_pro` dans KiCad 10. Toutes les sources ont été enregistrées par KiCad 10.0.6.

- **Schéma** : une feuille racine (XIAO, GNSS J4, extensions J6 et J8, Modulino Knob J7) et une sous-feuille, « Capteurs I2C et écran ».
- **PCB** :
  - **F.Cu** : uniquement les ponts de soudure du XIAO, côté écran ;
  - **B.Cu** : fils nus de la face dessous ;
  - **In1.Cu / In2.Cu** : fils isolés de la face dessous. Ce ne sont pas des couches réelles : elles séparent les fils qui se croisent pour le DRC ;
  - `PL1` porte les 280 trous ; `Cmts.User` porte les numéros de fils ;
  - la vue 3D s'ouvre avec Alt+3.
- Ne pas faire « Mettre à jour le PCB depuis le schéma » sans précaution : les pastilles de `PL1` portent leurs nets à la main.

Documentation : `../../perfboard_une_plaque_c5_a7.md`. Régénération : `scripts/generer_c5_a7.py --force`, ce qui **écrase toute édition manuelle**.
