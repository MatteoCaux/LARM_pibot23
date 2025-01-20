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
Dans le ROS2 workspace, il faut build les packages:
- `./pkg-tbot/bin/install`
- `colcon build`
- `source install/setup.sh`


## Challenge 1

### Objectifs

L'objectif du challenge 1 est de parcourir entièrement une zone fermée en évitant les obstacles présents dans cette zone. Pendant la navigation, le robot doit être capable de détecter des fantômes verts à l'aide d'une caméra.

### Lancement du challenge 1

Pour le challenge 1, vous pouvez : 
- Lancer le fichier `simulation_v1_launch.yaml` pour la partie simulation : `ros2 launch tutorial_pkg simulation_v1_launch.yaml`
- Lancer le fichier `tbot_v1_launch.yaml` pour la partie réelle : `ros2 launch tutorial_pkg tbot_v1_launch.yaml`

## Challenge 2

### Objectifs

L'objectif du challenge 2 est de reprendre les fonctionnalités du challenge 1 en y ajoutant de nouvelles caractéristiques. La détection des fantômes est plus fine si bien que les faux positifs sont évités et qu'un robot n'est détcté qu'une seule fois. Une carte de l'environnement est créée en direct et les fantômes détectés sont représentés dessus.

### Lancement du challenge 2

Pour le challenge 2, vous pouvez :
