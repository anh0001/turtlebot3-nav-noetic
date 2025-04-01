# Turtlebot3 Navigation - ROS Noetic

A ROS Noetic package for autonomous navigation of the Turtlebot3 robot in a Gazebo simulation environment. This package enables the Turtlebot3 to navigate through a standard world while avoiding obstacles using the move_base framework.

## Features

- Autonomous navigation with obstacle avoidance
- Integration with Gazebo simulation
- Support for Turtlebot3 models (Burger, Waffle, Waffle Pi)
- Visualization with RViz
- Remote access via noVNC
- Predefined patrol routes
- Custom obstacle avoidance behaviors
- Docker support for cross-platform compatibility

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

## Task 1: Autonomous Navigation with Obstacle Avoidance

This task demonstrates controlling the Turtlebot3 in Gazebo, keeping it moving while avoiding obstacles using the ROS1 navigation stack.

### Running the Simulation

1. Launch the master file that coordinates simulation and navigation:
   ```bash
   roslaunch turtlebot3-nav-noetic turtlebot3_master.launch
   ```

   This will:
   - Start Gazebo with the Turtlebot3 in the standard world
   - Launch the navigation stack with move_base
   - Initialize the robot at the specified position
   - Open RViz for visualization

2. Manual Navigation with 2D Nav Goal:
   Once the simulation is running, you can use RViz to set navigation goals:
   - In RViz, click the "2D Nav Goal" button
   - Click and drag on the map to set a goal pose (position and orientation)
   - The robot will plan a path and navigate to the goal while avoiding obstacles
   
   **[Video Demo: Manual Navigation with 2D Nav Goal]** - *This video demonstrates how to set navigation goals manually using RViz's 2D Nav Goal tool and shows the robot successfully navigating to the target while avoiding obstacles.*

3. Running the Patrol Mode:
   To have the robot automatically patrol between predefined waypoints:
   ```bash
   roslaunch turtlebot3-nav-noetic turtlebot3_patrol.launch
   ```

   The robot will continuously move between four predefined waypoints, navigating around obstacles. This demonstrates:
   - Creating autonomous patrol behaviors with multiple waypoints
   - Using move_base for path planning while patrolling
   - Implementing loop behaviors with configurable wait times
   - Handling goal completion and failure recovery
   
   **[Video Demo: Robot Patrol]** - *This video shows the Turtlebot3 autonomously patrolling between predefined waypoints, demonstrating the continuous navigation capability and obstacle avoidance during patrol mode.*

### Verification

To verify that the system is working correctly:

1. Check RViz visualization:
   - The robot model should be visible
   - Laser scan data should appear as green points
   - Local and global costmaps should be visible
   - Planned paths should appear when navigation goals are set

2. Observe the robot's movement:
   - It should navigate smoothly to goals
   - It should slow down or adjust its path when approaching obstacles
   - It should successfully reach each goal position

3. Check terminal output:
   - Navigation status messages should indicate successful path planning
   - Goal status updates should show when goals are reached

## Task 2: Dockerized Simulation with Web Interface

This task demonstrates a unique skill in robot development: creating a fully dockerized simulation environment with a web-based interface, allowing cross-platform development and testing of ROS applications.

### Features

1. Cross-Platform Compatibility:
   - Run the complete ROS/Gazebo simulation environment on macOS (including Apple Silicon) or Windows
   - No need to install ROS, Gazebo, or other dependencies natively
   - Perfect for development on non-Ubuntu systems

2. Web-Based Interface:
   - Access the full desktop environment through any modern web browser
   - Visualize Gazebo, RViz, and other graphical tools without X11 forwarding
   - Improve collaboration by providing easy access to the simulation environment

3. Custom Navigation Behaviors:
   - The `obstacle_avoidance.py` script demonstrates implementing custom navigation logic
   - The `patrol.py` script shows how to create autonomous patrol behaviors
   - These scripts can be easily extended or modified to create new robot behaviors

4. Running Predefined Goals with Custom Obstacle Avoidance:
   ```bash
   roslaunch turtlebot3-nav-noetic turtlebot3_predefined_goals.launch
   ```
   
   This launches the simulation with a custom obstacle avoidance algorithm that demonstrates how to:
   - Set predefined goal locations
   - Detect obstacles using laser scan data
   - Navigate safely to goals while handling edge cases
   
   This launch file is particularly unique because:
   - It implements its own obstacle detection and control logic
   - It provides a fallback mechanism if move_base fails
   - It demonstrates direct control of robot velocity commands
   - It showcases custom parameter tuning for different environments
   
   **[Video Demo: Predefined Goals Navigation]** - *This video demonstrates the robot navigating to predefined goal locations using the custom navigation system, highlighting the specialized obstacle avoidance algorithm in action.*
   
   **[Video Demo: Custom Obstacle Avoidance]** - *This detailed video focuses specifically on how the custom obstacle detection and control logic handles challenging situations, showing the advantages of the custom implementation compared to standard navigation.*

### Running the Dockerized Simulation

1. Start the Docker container:
   ```bash
   ./start_docker_env.sh
   ```
   or
   ```bash
   docker-compose up
   ```

2. Access the environment:
   - Open your web browser and navigate to `http://localhost:6080/vnc.html`
   - Click "Connect" to access the desktop environment

3. In the terminal inside the web interface, run:
   ```bash
   roslaunch turtlebot3-nav-noetic turtlebot3_master.launch
   ```

4. In another terminal tab, run one of the behavior scripts:
   ```bash
   # For patrol behavior
   roslaunch turtlebot3-nav-noetic turtlebot3_patrol.launch
   
   # OR for predefined goals with obstacle avoidance
   roslaunch turtlebot3-nav-noetic turtlebot3_predefined_goals.launch
   ```

### Benefits and Applications

1. Development Benefits:
   - Consistent environment across different operating systems
   - Simplified setup process for new developers
   - Easy to share and reproduce simulation results

2. Educational Benefits:
   - Students can run complex ROS simulations without installing ROS
   - Instructors can provide pre-configured environments
   - Workshops can focus on ROS concepts rather than installation issues

3. Project Collaboration:
   - Team members can use the same environment regardless of their OS
   - Demonstrations can be provided to stakeholders without ROS knowledge
   - CI/CD pipelines can be implemented for robotics projects

## Navigation Parameters

The navigation parameters can be adjusted in the config files:
- `config/costmap_common_params.yaml`: Common parameters for costmaps
- `config/global_costmap_params.yaml`: Global costmap parameters
- `config/local_costmap_params.yaml`: Local costmap parameters
- `config/move_base_params.yaml`: Move base parameters

## Project Structure

- `launch/`: Launch files for the simulation and navigation
  - `turtlebot3_master.launch`: Main launch file that starts both simulation and navigation
  - `turtlebot3_patrol.launch`: Launches patrol behavior
  - `turtlebot3_predefined_goals.launch`: Launches custom obstacle avoidance with predefined goals
- `config/`: Configuration files for move_base and costmaps
- `scripts/`: Python scripts for navigation behaviors
  - `obstacle_avoidance.py`: Custom obstacle avoidance implementation
  - `patrol.py`: Autonomous patrol behavior
  - `initial_pose_publisher.py`: Helper for setting robot's initial pose
- `rviz/`: RViz configuration
- `worlds/`: Gazebo world files
- `Dockerfile` and `docker-compose.yml`: Docker setup with noVNC

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

4. **Navigation problems**
   - If the robot cannot plan a path, try adjusting the costmap parameters
   - If move_base fails to start, check if all dependencies are installed

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- [ROS](https://www.ros.org/)
- [Turtlebot3](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
- [Navigation Stack](https://wiki.ros.org/navigation)