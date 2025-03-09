# Project Title:
Simple Planner

# Overview
This repository contains the implementation of the Simple Planner. The project was developed as part of the course **Robot Programming**, taught by Professor **Giorgio Grisetti**, as part of my Master’s in AI and Robotics at Sapienza University of Rome. 

## Demonstration Videos
## Vatican City Path Planner
[![Watch the video](https://img.youtube.com/vi/a_FXH-olXFI/maxresdefault.jpg)](https://youtu.be/a_FXH-olXFI)

## Maze Path Planner
[![Watch the video](https://img.youtube.com/vi/HI9JwL4_bSU/maxresdefault.jpg)](https://youtu.be/HI9JwL4_bSU)

# Functionality
This project implements the **A* Search Algorithm** to determine the optimal path while considering different heuristics and penalties for obstacles.

### Arguments
#### `h` (Heuristic Distance)
Defines the heuristic function used for path planning. Available options:
- `h='m'` → **Manhattan Distance**
- `h='e'` → **Euclidean Distance**
- `h='c'` → **Chebyshev Distance**
- **Default:** Manhattan Distance (`h='m'`)

#### `p` (Penalty for Obstacles)
This argument penalizes paths passing through obstacles by increasing the cost. The higher the penalty, the less likely A* will consider paths through obstacles.
- **Default:** `p=1000.0`

# Installation & Setup
### Prerequisites
- Install **ROS1 Noetic** on your Linux machine.

### Clone the Repository
```bash
git clone "https://github.com/Sameer-Ahmed7/Simple-Planner.git"
```

### Build the Package
```bash
cd Simple-Planner
source devel/setup.bash
cd src/
cd simple_path_planner/
```





