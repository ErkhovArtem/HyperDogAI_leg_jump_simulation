# HyperDogAI_leg_jump_simulation
# Project Title

## Overview
MPC-based control system that allows robot leg jump in Gazebo simulation.

- Course: Advanced Control Methods, Skoltech, 2024
- Team Members: Elizaveta Pestova, Danil Belov, Liaisan Safarova, Erkhov Artem, Khabibullin Batyr
- Final Presentation: https://docs.google.com/presentation/d/1fb4WBCsUHritptmiDhKOIJ5rtWpCEz4LfbT3K--IwfE/edit#slide=id.g2e0b4b040c3_3_2

---

## Table of Contents

- [Overview](#overview)
- [Problem Statement](#problem-statement)
- [Results](#results)
- [Run the Project](#run-the-project)
- [Other Section](#other-section)
- [Bibliography](#bibliography)
---
## Problem Statement
The main problem in our project is to create a system to control robot leg using MPC controller. Therefore task can be derived into several subtasks:

### Subsection (if any)
1. Create dynamic model of the leg.
2. Creating MPC controller considering robot leg as rigib body.
3. Test MPC in Gazebo simulation.
4. Implement simulation results on real stand.
---
## Results
MPC controller derived that allows leg jump in simulation. Test in Gazebo provided.

## Run the Project
Step-by-step instructions on how to replicate the results obtained in this project. This should be clear enough for someone with basic knowledge of the tools used to follow.

### Requirements
List of prerequisites, dependencies, and environment setup necessary to run the project.

### Setup and Installation
```
docker build -t batyr_simulation .
xhost +local:root
docker run -it --env="DISPLAY" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" batyr_simulation
```

### Running the Code

In first terminal:
```
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch leg_utils controller.launch
```
In second terminal:
```
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosservice call /gazebo/unpause_physics
```
In third terminal:
```
source /opt/ros/noetic/setup.bash
source devel/setup.bash
cd ~/danil_sw/src/leg/control/scripts
python3 MPC_jump.py
```

### Documentation
If available, provide links to the project documentation or instructions on how to generate it.

---

## Bibliography
(If applicable) This section includes references to papers, articles, and other resources that informed the project's approach and methodology.

- Reference 1
- Reference 2
- Reference 3

