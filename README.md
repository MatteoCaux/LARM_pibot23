# LARM_pibot23 <!-- omit from toc -->

## Auteurs

Mattéo CAUX 
Kristian BOUVET
PAul MILLEMANN

## Introduction
Ce répertoire est un package ROS2 pour le contrôle d'un robot Kobuki.

## Table des matières <!-- omit from toc -->
- [Auteurs](#auteurs)
- [Introduction](#introduction)
- [Installation](#installation)
  - [Elément requis](#elément-requis)
  - [Configuration](#configuration)
  - [Installation des packages](#installation-des-packages)
  - [Build des packages](#build-des-packages)
- [Challenge 1](#challenge-1)
  - [Objectifs](#objectifs)
  - [Lancement du challenge 1](#lancement-du-challenge-1)
- [Challenge 2](#challenge-2)
  - [Objectifs](#objectifs-1)
  - [Lancement du challenge 2](#lancement-du-challenge-2)


## Installation
### Elément requis
Les éléments suivants doivent être installés pour pouvoir faire fonctionner le code de ce répertoire :
- ROS2 Iron : https://docs.ros.org/en/iron/index.html
- ROS2 packages : `apt install ros-iron-slam-toolbox ros-iron-nav2-bringup`
- Python 3.10 : https://www.python.org/downloads/
- Python packages :
    * math, os, signal, sys, time (installed with python)
    * numpy
    * colcon-common-extensions
    * opencv-python
    * pyrealsense2
    * cvbridge3
  
  il faut ajouter remappings=[('/base_scan','/scan')]
  a la fin du launch file du stage, dans Launchdescription dans le Node
  path :ros_space/src/pkg-stage/stage_ros2/launch/stage.launch.py
</br>

> Commande :  
> `pip install numpy colcon-common-extensions opencv-python pyrealsense2 cvbridge3 scikit-image`

-  $`\textcolor{red}{\text{[OPTIONAL]}}`$ Teleop twist keyboard (to control manually the robot)

### Configuration
Ajouter les lignes suivantes à votre fichier `~/.bashrc` :
```
source /opt/ros/iron/setup.bash #Add ROS2 source
export ROS_DOMAIN_ID=23 #Setup domain ID
source ~/ros_space/install/local_setup.bash
```

### Installation des packages
1. Ouvrer un terminal dans votre dossier ROS2 workspace
1. Cloner les packages IMT Tbot: `git clone https://bitbucket.org/imt-mobisyst/pkg-tbot.git` and `git clone https://bitbucket.org/imt-mobisyst/pkg-tsim.git`
1. Cloner ce répertoire : `git clone https://github.com/MatteoCaux/LARM_pibot23.git`


### Build des packages
Dans le ROS2 workspace, il faut build les packages :
- `./pkg-tbot/bin/install`
- `colcon build`
- `source install/setup.sh`


## Challenge 1

### Objectifs

L'objectif du challenge 1 est de parcourir entièrement une zone fermée en évitant les obstacles présents dans cette zone. Pendant la navigation, le robot doit être capable de détecter des fantômes verts à l'aide d'une caméra.

### Lancement du challenge 1

Pour le challenge 1, vous pouvez : 
- Lancer le fichier `simulation_v1_launch.yaml` pour la partie simulation : `ros2 launch grp_pibot23 simulation_v1_launch.yaml`
- Lancer le fichier `tbot_v1_launch.yaml` pour la partie réelle : `ros2 launch grp_pibot23 tbot_v1_launch.yaml`


Certains fichier du challenge 1 ont été modifié pour accomplir le challenge 2, ainsi pour retrouver les fichiers du challenge 1, veuillez vous référez au commit `c645cce`. Moreover, each files useful for the challenge 1 as been rename with `_1` to differ them from the challenge 2 files.

## Challenge 2

### Objectifs

L'objectif du challenge 2 est de reprendre les fonctionnalités du challenge 1 en y ajoutant de nouvelles caractéristiques. La détection des fantômes est plus fine si bien que les faux positifs sont évités et qu'un robot n'est détcté qu'une seule fois. Une carte de l'environnement est créée en direct et les fantômes détectés sont représentés dessus.

### Lancement du challenge 2

Pour le challenge 2, vous pouvez :

- Lancer le fichier `simulation_v2_launch.yaml` pour la partie simulation : `ros2 launch grp_pibot23 simulation_v2_launch.yaml`
- Lancer le fichier `tbot_v2_launch.yaml` pour la partie réelle : `ros2 launch grp_pibot23 tbot_v2_launch.yaml`
- Lancer le fichier `operator_launch.yaml` pour la partie réelle : `ros2 launch grp_pibot23 operator_launch.yaml`
- - Lancer le fichier ?? pour la partie vision: ..

De nombreux topic sont créer contenant de nombreuse informations utiles :

`/is_pathfinding_on_move_to` stock une variable booléan qui définit si le node pathfinding_2 doit suivre un goal (etat : True) ou être en navigation aléatoire réactive (etat : False).

'/moveto/globalgoal' correspond à une variable Pose d'un objectif de position dans le repère '/map', '/moveto/globalgoalMarker' est le marqueur correspondant.
'/moveto/localgoal' correspond à une variable Pose d'un objectif de position dans le repère '/base_link du robot', '/moveto/localgoalMarker' est le marqueur correspondant.

'/map_prct_discovered' correspond à un pourcentage de la map découverte, et permet de choisir dans quelle mode de déplacement se trouve pathfinding.

'/is_manual_mode' correspond à une variable booléan utilisée par le node operator qui permet de dire à pathfinding d'arrêter ses maneuvres pour être en mode manuel.
'/pathfinding_msg' correspond à une variable String qui stock les messages de fonctionnement de pathfinding.

Le mode opérateur peut être acceder via le launch file 'operator_launch.yaml'

ensuite on utilise RQT reconfigure pour acceder aux paramètres modifiables:
operator_control: permet d'activer un mode manuel qui se commande ensuite avec le teleop ouvert, permet d'afficher les msg de pathfinding et le prct de map decouvert
global_goal_publisher_node: permet de publier des nouveaux goal

on peut aussi modifier des nodes precedemment lancer:
map_subscriber: modifier les prcts de map decouvert qui regit l'utilisation du mode random reactif ou mode move_to