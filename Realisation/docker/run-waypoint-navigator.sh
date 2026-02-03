#!/bin/bash

# TurtleBot 3 Waypoint Navigator - Docker Compose Helper Script
#
# This script simplifies running the waypoint navigator in Docker with custom parameters
#
# Usage:
#   ./run-waypoint-navigator.sh [OPTIONS]
#
# Options:
#   --speed SPEED                      Linear speed (m/s), default: 0.2
#   --angular-speed SPEED              Angular speed (rad/s), default: 0.5
#   --repetitions N                    Number of repetitions (0 = infinite), default: 1
#   --pattern PATTERN                  Pattern type (default|figure_eight|spiral), default: default
#   --position-tolerance TOL           Position tolerance (m), default: 0.1
#   --angle-tolerance TOL              Angle tolerance (rad), default: 0.1
#   --namespace NS                     ROS namespace, default: (empty)
#   --ros-domain-id ID                 ROS Domain ID, default: 0
#   --detach                           Run in background (detached mode)
#   --help                             Show this help message

set -e

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Default values
WAYPOINT_SPEED="0.2"
WAYPOINT_ANGULAR_SPEED="0.5"
WAYPOINT_REPETITIONS="1"
WAYPOINT_PATTERN="default"
WAYPOINT_POSITION_TOLERANCE="0.1"
WAYPOINT_ANGLE_TOLERANCE="0.1"
WAYPOINT_NAMESPACE=""
ROS_DOMAIN_ID="0"
DETACH_MODE=""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print help
print_help() {
    head -n 25 "$0" | tail -n 22
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --speed)
            WAYPOINT_SPEED="$2"
            shift 2
            ;;
        --angular-speed)
            WAYPOINT_ANGULAR_SPEED="$2"
            shift 2
            ;;
        --repetitions)
            WAYPOINT_REPETITIONS="$2"
            shift 2
            ;;
        --pattern)
            WAYPOINT_PATTERN="$2"
            shift 2
            ;;
        --position-tolerance)
            WAYPOINT_POSITION_TOLERANCE="$2"
            shift 2
            ;;
        --angle-tolerance)
            WAYPOINT_ANGLE_TOLERANCE="$2"
            shift 2
            ;;
        --namespace)
            WAYPOINT_NAMESPACE="$2"
            shift 2
            ;;
        --ros-domain-id)
            ROS_DOMAIN_ID="$2"
            shift 2
            ;;
        --detach)
            DETACH_MODE="-d"
            shift
            ;;
        --help)
            print_help
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            print_help
            exit 1
            ;;
    esac
done

# Print configuration
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}TurtleBot 3 Waypoint Navigator${NC}"
echo -e "${GREEN}========================================${NC}"
echo "Configuration:"
echo "  Speed: $WAYPOINT_SPEED m/s"
echo "  Angular Speed: $WAYPOINT_ANGULAR_SPEED rad/s"
echo "  Repetitions: $WAYPOINT_REPETITIONS"
echo "  Pattern: $WAYPOINT_PATTERN"
echo "  Position Tolerance: $WAYPOINT_POSITION_TOLERANCE m"
echo "  Angle Tolerance: $WAYPOINT_ANGLE_TOLERANCE rad"
[ -n "$WAYPOINT_NAMESPACE" ] && echo "  Namespace: $WAYPOINT_NAMESPACE"
echo "  ROS Domain ID: $ROS_DOMAIN_ID"
[ -n "$DETACH_MODE" ] && echo "  Mode: Detached (background)"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if workspace exists
if [ ! -d "$PROJECT_ROOT/turtlebotrossim" ]; then
    echo -e "${RED}ERROR: turtlebotrossim workspace not found${NC}"
    echo "Expected at: $PROJECT_ROOT/turtlebotrossim"
    exit 1
fi

# Check if the package exists
if [ ! -d "$PROJECT_ROOT/turtlebotrossim/src/turtlebot3_waypoint_navigator" ]; then
    echo -e "${RED}ERROR: turtlebot3_waypoint_navigator package not found${NC}"
    echo "Expected at: $PROJECT_ROOT/turtlebotrossim/src/turtlebot3_waypoint_navigator"
    exit 1
fi

# Check if workspace is built
if [ ! -f "$PROJECT_ROOT/turtlebotrossim/install/setup.sh" ]; then
    echo -e "${YELLOW}WARNING: Workspace not built yet${NC}"
    echo "The Docker container will attempt to build it automatically."
    echo ""
fi

# Export variables for docker-compose
export WAYPOINT_SPEED
export WAYPOINT_ANGULAR_SPEED
export WAYPOINT_REPETITIONS
export WAYPOINT_PATTERN
export WAYPOINT_POSITION_TOLERANCE
export WAYPOINT_ANGLE_TOLERANCE
export WAYPOINT_NAMESPACE
export ROS_DOMAIN_ID

# Run docker-compose
echo "Starting waypoint navigator..."
echo ""

cd "$PROJECT_ROOT"
docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up $DETACH_MODE waypoint_navigator

echo ""
echo -e "${GREEN}Done!${NC}"
