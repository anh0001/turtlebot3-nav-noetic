# Turtlebot3 Navigation - ROS Noetic

A ROS Noetic package for autonomous navigation of the Turtlebot3 robot in a Gazebo simulation environment. This package enables the Turtlebot3 to navigate through a standard world while avoiding obstacles using the move_base framework.

## Features

- Autonomous navigation with obstacle avoidance
- Integration with Gazebo simulation
- Support for Turtlebot3 models (Burger, Waffle, Waffle Pi)
- Visualization with RViz
- Remote access via noVNC

## Requirements

- Ubuntu 20.04 (Focal Fossa)
- ROS Noetic
- Gazebo 11
- Turtlebot3 packages
- Python 3

## Setup Instructions

### Option 1: Local Installation

1. Install ROS Noetic following the [official installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu)

2. Install Turtlebot3 packages:
   ```bash
   sudo apt-get install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations ros-noetic-navigation
   ```

3. Create a catkin workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/src
   git clone https://github.com/anh0001/turtlebot3-nav-noetic.git
   cd ..
   catkin_make
   source devel/setup.bash
   ```

4. Set Turtlebot3 model:
   ```bash
   echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
   source ~/.bashrc
   ```

### Option 2: Docker with noVNC (for macOS with Apple Silicon)

1. Install Docker and Docker Compose on your macOS system.

2. Clone this repository:
   ```bash
   git clone https://github.com/anh0001/turtlebot3-nav-noetic.git
   cd turtlebot3-nav-noetic
   ```

3. Start the Docker container:
   ```bash
   docker-compose up
   ```

4. Access the environment via noVNC:
   - Open a web browser and navigate to `http://localhost:6080`

## Usage

### Launch the Simulation

```bash
# Launch Gazebo with Turtlebot3 in the standard world
roslaunch turtlebot3-nav-noetic turtlebot3_simulation.launch

# In a new terminal, launch the navigation stack
roslaunch turtlebot3-nav-noetic turtlebot3_navigation.launch
```

### Running the Obstacle Avoidance Node

```bash
# In a new terminal
rosrun turtlebot3-nav-noetic obstacle_avoidance.py
```

### Creating Maps with SLAM

```bash
# Create maps directory
mkdir -p ~/catkin_ws/src/turtlebot3-nav-noetic/maps

# First run SLAM to create a map (in a new terminal)
roslaunch turtlebot3_slam turtlebot3_slam.launch

# Drive the robot around to map the environment (in another terminal)
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# When mapping is complete, save the map
rosrun map_server map_saver -f ~/catkin_ws/src/turtlebot3-nav-noetic/maps/map
```

## Docker Environment Details

The Docker environment includes:
- Ubuntu 20.04
- ROS Noetic
- Gazebo 11
- Turtlebot3 packages
- RViz
- noVNC for remote desktop access

## Navigation Parameters

The navigation parameters can be adjusted in the config files:
- `config/costmap_common_params.yaml`: Common parameters for costmaps
- `config/global_costmap_params.yaml`: Global costmap parameters
- `config/local_costmap_params.yaml`: Local costmap parameters
- `config/move_base_params.yaml`: Move base parameters

## Project Structure

- `launch/`: Launch files for the simulation and navigation
- `config/`: Configuration files for move_base and costmaps
- `scripts/`: Python scripts for obstacle avoidance
- `rviz/`: RViz configuration
- `worlds/`: Gazebo world files
- `Dockerfile` and `docker-compose.yml`: Docker setup with noVNC

## Verification

To verify that the system is working correctly:

1. Launch the simulation and navigation as described above
2. Run the obstacle avoidance node
3. In RViz, you should see:
   - The robot model
   - Laser scan data
   - Costmaps (global and local)
   - Planned path

The robot should move autonomously while avoiding obstacles in the environment.

## Troubleshooting

### Common Issues

1. **"Error in connection to Gazebo"**
   - Ensure that Gazebo is properly installed
   - Try resetting the Gazebo environment variables:
     ```bash
     killall gzserver gzclient
     export GAZEBO_MASTER_URI=http://localhost:11345
     ```

2. **"Could not find the Turtlebot3 model"**
   - Verify that the TURTLEBOT3_MODEL environment variable is set:
     ```bash
     echo $TURTLEBOT3_MODEL
     ```
   - It should return "burger", "waffle", or "waffle_pi"

3. **noVNC connection issues**
   - Ensure port 6080 is not being used by another application
   - Check Docker logs for any errors:
     ```bash
     docker-compose logs
     ```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- [ROS](https://www.ros.org/)
- [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Navigation Stack](https://wiki.ros.org/navigation)