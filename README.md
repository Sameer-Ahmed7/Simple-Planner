# Project Title:
Simple Planner

# Overview
This repository contains the implementation of the Simple Planner. The project was developed as part of the course **Robot Programming**, taught by Professor **Giorgio Grisetti**, as part of my Master’s in AI and Robotics at Sapienza University of Rome. 

This project focuses on finding an optimal path between two points starting point and goal point while avoiding any obstacles. The primary goal is to create a clear, obstacle-free path between the two points. To achieve this, the A* search algorithm is used to find the most efficient path.


## Directory Structure
```
📦 Simple-Planner
├── 📂 assets  
│   └── ...  --> Images of maps
├── 📂 build
│   └── ...  --> Build files generated during the build process.
├── 📂 devel
│   └── ...  --> Development files and environment setup.
├── 📂 src
│   └── CMakeLists.txt    --> CMake configuration for the overall project.
│   └── simple_path_planner
│       ├── CMakeLists.txt  --> CMake configuration for the path planner package.
│       ├── package.xml    --> ROS package manifest.
│       ├── planner.launch --> Launch file to start the path planner node.
│       ├── maps
│       │   └── vatican_image.png  --> Map image used for path planning.
│       │   └── maze.png  --> Map image used for path planning.
│       └── src
│           └── path_planner_node.cpp  --> Implementation of the path planner node.
├── .catkin_workspace  --> Indicates this directory is a catkin workspace.
└── README.md          --> Documentation for the repository.
```
## Demonstration Videos
### Vatican City Path Planner
[![Watch the video](https://img.youtube.com/vi/a_FXH-olXFI/maxresdefault.jpg)](https://youtu.be/a_FXH-olXFI)

### Maze Path Planner
[![Watch the video](https://img.youtube.com/vi/HI9JwL4_bSU/maxresdefault.jpg)](https://youtu.be/HI9JwL4_bSU)

# Functionality
This project implements the **A* Search Algorithm** to determine the optimal path while considering different heuristics and penalties for obstacles.

## Features
- **Valid Start and Goal Points**: The starting point and goal point cannot be placed on obstacles; they must be placed on free space.
- **Dynamic Path Update**: If the starting point or goal point is changed, the path is automatically updated in real-time.

## Arguments
### `h` (Heuristic Distance)
Defines the heuristic function used for path planning. Available options:
- `h='m'` → **Manhattan Distance**
- `h='e'` → **Euclidean Distance**
- `h='c'` → **Chebyshev Distance**
- **Default:** Manhattan Distance (`h='m'`)

### `p` (Penalty for Obstacles)
This argument penalizes paths passing through obstacles by increasing the cost. The higher the penalty, the less likely A* will consider paths through obstacles.
- **Default:** `p=1000.0`

# Installation & Setup
## Prerequisites
- Install **ROS1 Noetic** on your Linux machine.

## Clone the Repository
```bash
git clone "https://github.com/Sameer-Ahmed7/Simple-Planner.git"
```

## Build the Package
```bash
cd Simple-Planner
source devel/setup.bash
cd src/
cd simple_path_planner/
```
## Running the Planner
### Start ROS Core
Open a new terminal and run:
```bash
roscore
```

### Launch the Path Planner
Since a launch file is provided, all necessary components (map server, RViz, etc.) will start automatically:
```bash
roslaunch simple_path_planner planner.launch h:='m' p:='1000'
```
*Modify `h` and `p` values as needed.*

# Customizing the Map
The default map used is **vatican_image.png**. If you want to use a different map:
1. Add the new map image to the `map` folder.
2. Edit the `.yaml` file and change `vatican_image.png` to your desired map file.









