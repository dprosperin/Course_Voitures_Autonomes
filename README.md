# Projet de Voiture Autonome - COVAPSY 🚗


Ce dépôt contient le code développé par l'équipe **GE1** en deuxième année du BUT GEII de l'IUT de Cachan pour concevoir une voiture autonome dans le cadre de la compétition COVAPSY. Le règlement officiel de la course est disponible [ici](https://ajuton-ens.github.io/CourseVoituresAutonomesSaclay/).

---
## Organisation du dépôt

Voici la structure des fichiers et répertoires principaux du dépôt :

```plaintext
├── code_carte_moteur/             # Code source de la carte actionneur
├── capteur_obstacles/          # Code source de la carte capteur : capteurs d'ostcles et connecteur bluetooth
├── code_carte_strategie_lidar/      # Code source de la carte stratégie et LiDAR
├── README.md                      # Documentation principale
```

# Caractéristiques de la voiture​
![Diagramme de définition de bloc SysML de la voiture autonome​](./images/copvasy_diagrame_exigence.png)

Diagramme de définition de bloc SysML de la voiture autonome​

#  Architecture globale : Le schéma synoptique 
![Schéma synoptique de communication​​](./images/schema_synoptique_communication.png)

Schéma synoptique de communication​

## TODO
- [x] Intégration du connecteur Bluetooth
- [ ] Programmation et intégration de la fourche optique
- [ ] Programmation et intégration des capteurs d'obstacles arrières
- [ ] Optimisation du décodage des trames LiDAR
- [ ] Révision de l'algorithme de conduite autonome

## Guide d'Installation
### Logiciels et pilotes:
  - [STM32CubeIDE 1.17.0](https://www.st.com/en/development-tools/stm32cubeide.html) pour la programmation embarquée.
  - Python 3.8+ avec la bibliothèque matplotlib pour la simulation de la voiture codée en Python
  - Les scripts de simulation de la voiture fonctionnent uniquement dans la version [Webots 2023b](https://github.com/cyberbotics/webots/releases/tag/R2023b) 
  - Programme de configuration des capteurs d'obstacles *NAssitant*, pilote USB to UART bridge et exemples d'implémentation de pilotes du Tofsense : [Page de téléchargement de Nooploop](https://www.nooploop.com/download/)
  - Programme de configuration des Herkulex en français : [Installateur de Herkulex Manager en français](https://upsud-my.sharepoint.com/:u:/g/personal/david_prosperin_universite-paris-saclay_fr/EcTWrL1j4UJJgb2gJz-qEDoBWvkneA_PfRN32brg2IUPKg?e=HXQQ5R)
  - Programme d'analyse et test des trames en CAN : [BUSMASTER](https://rbei-etas.github.io/busmaster/)
  - Pilote pour le convertisseur IXXAT USB-TO-CAN : [Driver VCI V4](https://www.hms-networks.com/support/general-downloads)
  - Bibliothèque logiciel permettant d'utiliser le convertisseur IXXAT USB-TO-CAN avec Bus Master. Il suffit de placer le fichier [CAN_IXXAT_VCI.dll](https://upsud-my.sharepoint.com/:u:/g/personal/pauline_michel1_universite-paris-saclay_fr/EYL0c9kcsmpOkCqYnstpeuUBmyhb8S_0AUN9MwtDM4tnOg?e=cGdKdd) dans le dossier racine de Bus Master 
  - Application mobile permettant de communiquer en bluetooth avec la voiture [BlueDuino](https://play.google.com/store/apps/details?id=com.app.aktham.blueduino&pcampaignid=web_share). On utilise le joystik.
  - Terminal simple de port série : [CoolTerm](https://freeware.the-meiers.org/)
  - Ce projet utilise le formatage de la visionneuse de télémétrie Teleplot. L'extension vscode fonctionne très bien : [Extension Teleplot](https://marketplace.visualstudio.com/items?itemName=alexnesnes.teleplot)
  - Génération de la documentation [Doxygen](https://www.doxygen.nl/manual/index.html)
### Étapes
1. Clonez le dépôt Git :
```bash
git clone https://github.com/dprosperin/Course_Voitures_Autonomes.git
cd Course_Voitures_Autonomes
```

2.	Changez de branche selon vos besoins :
```bash
git checkout <nom_de_la_branche>
```
3.	Suivez les instructions spécifiques à chaque branche dans leurs sous-dossiers respectifs.

## Documents techniques
### Interface LiDAR
- Protocole d'interface et notes d'application appliqué aux séries RPLIDAR A et S : [Lien vers le site de SLAMTEC](https://bucket-download.slamtec.com/6494fd238cf5e0d881f56d914c6d1f355c0f582a/LR001_SLAMTEC_rplidar_protocol_v2.4_en.pdf)
- [RPLidar S2m1-R2 datasheet](https://cdn.robotshop.com/media/r/rpk/rb-rpk-20/pdf/rplidar-s2-360-laser-scanner-30-m-datasheet.pdf)
### Base motrice
- Manuel utilisateur du Herkulex DRS-0101 [Lien vers manuel](https://cdn.robotshop.com/media/d/das/rb-das-05/pdf/_eng_herkulex_manual_20140218.pdf)
- Lien vers le manuel du Pololu G2 High-Power Motor Driver 18v17 [lien page du produit sur le site de pololu](https://www.pololu.com/product/2991)
### Capteurs d'obstacles et connecteur bluetooth
- Manuel utilisateur du capteurs d'obstacles Nooploop TOFSense : [lien vers le manuel des modèles TOFSense, TOFSense-UART et TOFSense S](https://ftp.nooploop.com/downloads/tofsense/TOFSense_User_Manual_V3.0_en.pdf)
- Manuel de référence du connecteur bluetooth [PmodBT2™ Reference Manual](https://digilent.com/reference/_media/reference/pmod/pmodbt2/pmodbt2_rm.pdf)
- Module de données Bluetooth Référence des commandes et informations avancées Guide de l'utilisateur [Lien vers le manuel](https://ww1.microchip.com/downloads/en/DeviceDoc/bluetooth_cr_UG-v1.0r.pdf)

# Développé par
* __David PROSPERIN__ : <david.prosperin@universite-paris-saclay.fr>
* __Mateo MUNOZ__ : <joseph.munoz-saltos@universite-paris-saclay.fr>
* __Yassine MESBAHI__ : <yassine.mesbahi@universite-paris-saclay.fr>