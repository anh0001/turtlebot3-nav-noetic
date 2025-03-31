#!/bin/bash

# Script to test the Turtlebot3 navigation functionality

# Colors for terminal output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Testing Turtlebot3 Navigation Functionality...${NC}"

# Check if ROS is installed and sourced
if ! command -v roscore &> /dev/null; then
    echo -e "${RED}ROS is not installed or not sourced. Please install ROS and source setup.bash.${NC}"
    echo -e "Try: source /opt/ros/noetic/setup.bash"
    exit 1
fi

# Check if turtlebot3 model is set
if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo -e "${YELLOW}TURTLEBOT3_MODEL is not set. Setting to 'burger'.${NC}"
    export TURTLEBOT3_MODEL=burger
fi

# Function to check if a ROS node is running
check_ros_node() {
    local node_name=$1
    rosnode list | grep -q "$node_name"
    return $?
}

# Function to check if a ROS topic is being published
check_ros_topic() {
    local topic_name=$1
    rostopic info "$topic_name" 2>&1 | grep -q "Publishers:"
    return $?
}

echo -e "${GREEN}Step 1: Checking if roscore is running...${NC}"
if ! check_ros_node "/rosout"; then
    echo -e "${YELLOW}Starting roscore...${NC}"
    gnome-terminal -- roscore &
    sleep 3
fi

if check_ros_node "/rosout"; then
    echo -e "${GREEN}✓ roscore is running.${NC}"
else
    echo -e "${RED}✗ Failed to start roscore. Please start it manually.${NC}"
    exit 1
fi

echo -e "${GREEN}Step 2: Checking if Gazebo is running...${NC}"
if ! check_ros_node "/gazebo"; then
    echo -e "${YELLOW}Starting Gazebo with Turtlebot3...${NC}"
    gnome-terminal -- roslaunch turtlebot3-nav-noetic turtlebot3_simulation.launch &
    sleep 10
fi

if check_ros_node "/gazebo"; then
    echo -e "${GREEN}✓ Gazebo is running.${NC}"
else
    echo -e "${RED}✗ Failed to start Gazebo. Please start it manually:${NC}"
    echo -e "${YELLOW}   roslaunch turtlebot3-nav-noetic turtlebot3_simulation.launch${NC}"
    exit 1
fi

echo -e "${GREEN}Step 3: Checking if the Turtlebot3 model is loaded...${NC}"
if check_ros_topic "/scan"; then
    echo -e "${GREEN}✓ Turtlebot3 model is loaded in Gazebo.${NC}"
else
    echo -e "${RED}✗ Turtlebot3 model is not properly loaded.${NC}"
    exit 1
fi

echo -e "${GREEN}Step 4: Starting navigation stack...${NC}"
gnome-terminal -- roslaunch turtlebot3-nav-noetic turtlebot3_navigation.launch &
sleep 8

if check_ros_node "/move_base"; then
    echo -e "${GREEN}✓ Navigation stack is running.${NC}"
else
    echo -e "${RED}✗ Failed to start navigation stack. Please start it manually:${NC}"
    echo -e "${YELLOW}   roslaunch turtlebot3-nav-noetic turtlebot3_navigation.launch${NC}"
    exit 1
fi

echo -e "${GREEN}Step 5: Starting obstacle avoidance node...${NC}"
gnome-terminal -- rosrun turtlebot3-nav-noetic obstacle_avoidance.py &
sleep 3

if check_ros_node "/turtlebot_navigator"; then
    echo -e "${GREEN}✓ Obstacle avoidance node is running.${NC}"
else
    echo -e "${RED}✗ Failed to start obstacle avoidance node. Please start it manually:${NC}"
    echo -e "${YELLOW}   rosrun turtlebot3-nav-noetic obstacle_avoidance.py${NC}"
    exit 1
fi

echo -e "${GREEN}Step 6: Checking if the robot is moving...${NC}"
echo -e "${YELLOW}Monitoring cmd_vel topic for 5 seconds to verify robot movement...${NC}"
movement_detected=false
timeout 5 rostopic echo /cmd_vel -n 5 > /tmp/cmd_vel_output.txt

if grep -q "linear" /tmp/cmd_vel_output.txt; then
    movement_detected=true
fi

if [ "$movement_detected" = true ]; then
    echo -e "${GREEN}✓ Robot is moving! Navigation is working correctly.${NC}"
else
    echo -e "${RED}✗ No movement detected. There might be an issue with the navigation stack.${NC}"
    echo -e "${YELLOW}   Check the terminal output of the obstacle_avoidance.py script for errors.${NC}"
fi

echo -e "${GREEN}Test completed!${NC}"
echo -e "${YELLOW}The following nodes are now running:${NC}"
rosnode list

echo -e "${YELLOW}To stop all ROS nodes, run: rosnode kill -a${NC}"
echo -e "${YELLOW}To monitor robot movement: rostopic echo /cmd_vel${NC}"
echo -e "${YELLOW}To visualize in RViz: rviz -d $(rospack find turtlebot3_navigation)/rviz/turtlebot3_navigation.rviz${NC}"