


# Simulation des robots mobiles ENSTA Bretagne


<img src="https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white" /> <img src="https://img.shields.io/badge/Lua-2C2D72?style=for-the-badge&logo=lua&logoColor=white" /> 

Simulation DDBoat dynamique sur VREP.

#### Date: Avril 2021
#### Auteurs
* **Estelle Arricau** _alias_ [@estellearrc](https://github.com/estellearrc)
* **Antonin Betaille** _alias_ [@Anton1B](https://github.com/Anton1B)
* **Jean-Vincent Klein** _alias_ [@jeanvins57](https://github.com/jeanvins57)
* **Nicolas Odorico** _alias_ [@Nicoodo](https://github.com/Nicoodo)
* **Hugo Piquard** _alias_ [@hugoPiq](https://github.com/hugoPiq)

#### Encadrant
* **Benoit Zerr**

#### Pré-requis
VREP Version 3.6.2

ROS Noetic (remarque: la version Melodic fonctionne également).

Python Version 3 minimum.

#### Installation

- Cloner le dépot.

#### Démarrage
- Lancer ROSCORE en premier:
_exemple_: Executez la commande ``roscore`` .
- Lancer VREP puis ouvrir la scène ``real_boat.ttt``.
- Lancer ``controler.py`` .
- Lancer la simulation sur VRP.

#### Commande
Le fichier ``controler.py`` est un exemple de controleur proportionnel suivie de cap. Dans cet exemple, le cap suivi est de -PI/4.










