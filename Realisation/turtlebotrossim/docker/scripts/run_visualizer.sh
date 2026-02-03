#!/bin/bash
################################################################################
# Map Visualizer Runner Script
#
# This script runs the live command-line map visualizer for TurtleBot3 navigation.
# It sources the ROS environment and starts the Python visualizer.
#
# Usage:
#   ./run_visualizer.sh
#
################################################################################

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}╔════════════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║          TurtleBot3 Live Map Visualizer - Starting...                 ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════════════╝${NC}"
echo ""

# Source ROS environment
echo -e "${CYAN}[1/3]${NC} Sourcing ROS environment..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓${NC} ROS Humble environment sourced"
else
    echo -e "${RED}✗${NC} ROS environment not found!"
    exit 1
fi

# Source workspace if it exists
if [ -f /workspace/install/setup.bash ]; then
    source /workspace/install/setup.bash
    echo -e "${GREEN}✓${NC} Workspace environment sourced"
fi

# Check if script exists
SCRIPT_PATH="/opt/scripts/map_visualizer.py"
echo -e "${CYAN}[2/3]${NC} Checking visualizer script..."
if [ ! -f "$SCRIPT_PATH" ]; then
    echo -e "${RED}✗${NC} Visualizer script not found at: $SCRIPT_PATH"
    exit 1
fi
echo -e "${GREEN}✓${NC} Visualizer script found"

# Check ROS connectivity
echo -e "${CYAN}[3/3]${NC} Checking ROS connectivity..."
if ! timeout 5 ros2 node list &>/dev/null; then
    echo -e "${YELLOW}⚠${NC} Warning: Could not detect ROS nodes (may still work)"
else
    echo -e "${GREEN}✓${NC} ROS nodes detected"
fi

echo ""
echo -e "${CYAN}════════════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}Starting Map Visualizer...${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${YELLOW}Press Ctrl+C to exit${NC}"
echo ""

# Run the visualizer
exec python3 "$SCRIPT_PATH"
