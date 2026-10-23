# Provenance de cette édition

Cette édition est un instantané sélectionné du dépôt de travail FieldMind. La première édition (21 octobre 2026) reprenait l'arbre de travail, modifications locales comprises. La mise à jour du 23 octobre 2026 reprend le logiciel du rover, le firmware ESP32-C5 et la CAO imprimable depuis un commit du dépôt source, sans modification locale non commitée. Un test de sécurité dont l'obstacle tombait exactement sur la distance d'arrêt absolu (résultat dépendant de l'arrondi flottant sous Linux) a été rapproché à 0,62 m. Elle ne reprend pas l'historique du dépôt de travail privé.

Le manifeste [`snapshot-source.json`](snapshot-source.json) indique le commit source de référence, l'heure de capture et les empreintes des fichiers source copiés avant les adaptations de publication. Les empreintes de source ne sont donc pas celles de tous les fichiers publiés : les paramètres réseau, le profil IA du démarrage rapide, les chemins CAO et la documentation ont été adaptés dans cette édition.

## Sélection

Conservés : logiciel courant du rover, firmware C5, tests et pont ROS 2, première simulation web, code de déploiement, pièces imprimables courantes, plans, scripts Fusion, carte capteurs A7 et études KiCad utiles.

Écartés : accès machines et comptes, clés, mails du Challenge, commandes et adresses de livraison, données opérateur, journaux bruts, environnements Python, builds ESP, ancienne supervision remplacée, captures vidéo, rushes, `.blend`, anciens essais CAO et géométries externes aux droits non vérifiés. Les publications et services existants ne sont pas redéployés par ce dépôt.

Écartés en plus le 23 octobre : notes détaillées d'analyse de l'application du drone, bilan de travail interne, configuration de passerelle et de tunnel du GX10, rendu CAO contenant un modèle de servo tiers. Adaptations propres à cette édition : adresses du rover et du réseau remplacées par des exemples, nom d'un réseau domestique retiré de la liste de réseaux ignorés du firmware, et correction du dossier de photos par défaut (`mission/executeur.py`) pour que les tests n'écrivent pas dans le dossier de l'utilisateur.

Le démarrage rapide impose le profil IA `aucun`, les adresses de réseau sont des exemples et les services GX10 se lient à la boucle locale par défaut. Une utilisation depuis un rover nécessite de configurer explicitement l'adresse, la clé et le réseau privé appropriés.

## Images et vidéo

La vue CAO du README vient du projet ; ce n'est pas une photographie d'un rover assemblé. Les futures photos, vidéo des tests et démonstrations mesurées pourront compléter le README et le portfolio. Aucun lien factice n'est affiché à leur place.
