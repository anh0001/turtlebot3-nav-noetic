#!/bin/bash

# Script to start the Docker environment with noVNC for remote display

# Colors for terminal output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Starting Turtlebot3 Navigation Docker Environment...${NC}"

# Check if Docker is installed and running
if ! command -v docker &> /dev/null; then
    echo -e "${YELLOW}Docker is not installed. Please install Docker first.${NC}"
    exit 1
fi

if ! docker info &> /dev/null; then
    echo -e "${YELLOW}Docker is not running. Please start Docker first.${NC}"
    exit 1
fi

# Check if docker-compose is installed
if ! command -v docker-compose &> /dev/null; then
    echo -e "${YELLOW}Docker Compose is not installed. Please install Docker Compose first.${NC}"
    exit 1
fi

# Build and start the Docker container
echo -e "${GREEN}Building and starting the Docker container...${NC}"
docker-compose up -d

# Wait for container to start
echo -e "${GREEN}Waiting for the container to start...${NC}"
sleep 5

# Check if the container is running
CONTAINER_ID=$(docker-compose ps -q)
if [ -z "$CONTAINER_ID" ]; then
    echo -e "${YELLOW}Container failed to start. Check the logs with 'docker-compose logs'.${NC}"
    exit 1
fi

# Get the container IP
CONTAINER_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' $CONTAINER_ID)

echo -e "${GREEN}Docker container started successfully!${NC}"
echo -e "${BLUE}==================================================================${NC}"
echo -e "${GREEN}Access the Turtlebot3 environment via noVNC:${NC}"
echo -e "${YELLOW}URL:${NC} http://localhost:6080/vnc.html"
echo -e "${YELLOW}Password:${NC} rosdev"
echo -e "${BLUE}==================================================================${NC}"
echo -e "${GREEN}To run the simulation, execute these commands in the container terminal:${NC}"
echo -e "${YELLOW}1. Launch Gazebo:${NC}"
echo -e "   roslaunch turtlebot3-nav-noetic turtlebot3_simulation.launch"
echo -e "${YELLOW}2. In a new terminal, launch the navigation stack:${NC}"
echo -e "   roslaunch turtlebot3-nav-noetic turtlebot3_navigation.launch"
echo -e "${YELLOW}3. In another terminal, run the obstacle avoidance node:${NC}"
echo -e "   rosrun turtlebot3-nav-noetic obstacle_avoidance.py"
echo -e "${BLUE}==================================================================${NC}"
echo -e "${GREEN}To stop the container:${NC} docker-compose down"
echo -e "${BLUE}==================================================================${NC}"

# Keep the script running to show logs (optional)
echo -e "${GREEN}Showing container logs (press Ctrl+C to exit logs but keep container running):${NC}"
docker-compose logs -f