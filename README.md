
# Ackermann Robot Project

## Overview
This project is a simulation and control implementation of an Ackermann steering robot using ROS 2. The project aims to demonstrate navigation capabilities and implement algorithms like Dijkstra's and Stochastic Model Predictive Control (MPC).

## Table of Contents
- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Navigation](#navigation)
- [Behavior Tree](#behavior-tree)
- [Algorithms](#algorithms)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Introduction
The Ackermann robot is designed to navigate in an indoor environment using LIDAR and other sensors.
This project integrates various components, including a URDF model, launch files, and navigation parameters.

## Features
- **Ackermann Steering Model**: A simple implementation of the Ackermann steering geometry.
- **Navigation Stack**: Integration with the ROS 2 Navigation stack for path planning and obstacle avoidance.
- **Behavior Tree**: A custom behavior tree to control navigation using the Navigate to Pose action server.
- **Algorithms**: Implementations of Dijkstra's algorithm and Stochastic Model Predictive Control for optimal path planning.

## Installation
To get started with this project, follow these steps:

1. **Clone the Repository**
   ```bash
   git clone https://github.com/RahulRNandan/Achermann_robot.git
   cd Achermann_robot
   ```

2. **Install Dependencies**
   Make sure you have ROS 2 installed. Then, install the required packages:
   ```bash
   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

3. **Build the Package**
   Inside your ROS 2 workspace, build the package:
   ```bash
   colcon build
   ```

## Usage
To launch the simulation and navigation stack, use the following command:
```bash
ros2 launch my_robot ackermann_launch.py
```

## Navigation
The navigation parameters can be modified in the `nav2_params.yaml` file located in the `params` directory. Adjust the settings for the planners and controllers according to your needs.

## Behavior Tree
The project includes a custom behavior tree for managing the robot's navigation tasks. The behavior tree can be found in the `behavior_trees` directory. You can modify the tree to add or change the robot's behaviors.

## Algorithms
### Dijkstra's Algorithm
This implementation can be found in `dijkstra_node.cpp`. It calculates the shortest path between two points in a given environment.

### Stochastic Model Predictive Control (MPC)
The MPC implementation can be found in `smpc_node.cpp`. It uses optimization techniques to determine the best control actions for the robot in a dynamic environment.

## Contributing
Contributions are welcome! If you would like to contribute to this project, please follow these steps:

1. Fork the repository
2. Create a new branch (`git checkout -b feature/YourFeature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add some feature'`)
5. Push to the branch (`git push origin feature/YourFeature`)
6. Open a Pull Request

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.

## Acknowledgements
- ROS 2 community for their continuous support and development.
- Tutorials and documentation available online for ROS 2 and behavior trees.


### Instructions for Creating the `README.md`

1. **Create the File**: In your project directory, create a file named `README.md`:
   ```bash
   touch README.md
   ```

2. **Edit the File**: Open the file in your preferred text editor and paste the above content. Adjust any sections as needed to fit your project's specifics.

3. **Commit the Changes**: After creating and editing the `README.md`, don't forget to add it to your Git repository and commit your changes:
   ```bash
   git add README.md
   git commit -m "Add detailed README.md"
   git push origin <branch-name>
   ```

## Project Status
**Note:** This project is currently a prototype and is not fully executable. It is intended for study and development purposes only. Users may encounter incomplete features or bugs. Contributions and suggestions for improvements are welcome!

