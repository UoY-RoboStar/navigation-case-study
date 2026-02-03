#!/bin/bash

# Build TurtleBot 3 Waypoint Navigator in Docker Container
#
# This script builds the waypoint navigator package inside a Docker container
# using the existing docker-compose infrastructure.
#
# Usage:
#   ./docker/build-waypoint-navigator.sh [OPTIONS]
#
# Options:
#   --clean              Clean build and install directories before building
#   --rebuild            Force rebuild (clean + build)
#   --help               Show this help message

set -e

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Default options
CLEAN_BUILD=false

# Function to print help
print_help() {
    head -n 15 "$0" | tail -n 12
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --clean|--rebuild)
            CLEAN_BUILD=true
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

# Print header
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}Waypoint Navigator - Docker Build${NC}"
echo -e "${BLUE}============================================${NC}"
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

echo -e "${GREEN}✓${NC} Workspace found"
echo -e "${GREEN}✓${NC} Package source found"
echo ""

# Clean build directories if requested
if [ "$CLEAN_BUILD" = true ]; then
    echo -e "${YELLOW}→${NC} Cleaning build directories..."

    if [ -d "$PROJECT_ROOT/turtlebotrossim/build/turtlebot3_waypoint_navigator" ]; then
        rm -rf "$PROJECT_ROOT/turtlebotrossim/build/turtlebot3_waypoint_navigator"
        echo -e "${GREEN}✓${NC} Cleaned build directory"
    fi

    if [ -d "$PROJECT_ROOT/turtlebotrossim/install/turtlebot3_waypoint_navigator" ]; then
        rm -rf "$PROJECT_ROOT/turtlebotrossim/install/turtlebot3_waypoint_navigator"
        echo -e "${GREEN}✓${NC} Cleaned install directory"
    fi

    echo ""
fi

# Export ROS_DOMAIN_ID for consistency
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo -e "${CYAN}Build Configuration:${NC}"
echo "  ROS Domain ID: $ROS_DOMAIN_ID"
echo "  Clean Build: $CLEAN_BUILD"
echo "  Workspace: turtlebotrossim"
echo "  Package: turtlebot3_waypoint_navigator"
echo ""

# Build using Docker
echo -e "${YELLOW}→${NC} Starting Docker build container..."
echo ""

cd "$PROJECT_ROOT/turtlebotrossim"

# Run the build in a temporary container using existing docker-compose
# Mount the current directory (turtlebotrossim) to /ws in the container
# Run as root to avoid permission issues with build/install directories
docker compose -f docker/docker-compose.yaml run --rm \
    --user root \
    -v "$(pwd):/ws:rw" \
    -e ROS_DOMAIN_ID=${ROS_DOMAIN_ID} \
    -e TURTLEBOT3_MODEL=waffle \
    devnogpu \
    bash -c "
        set -e

        echo '============================================'
        echo 'Building Waypoint Navigator Package'
        echo '============================================'
        echo ''

        echo '→ Sourcing ROS 2 environment...'
        if [ -f /opt/ros/humble/setup.sh ]; then
            source /opt/ros/humble/setup.sh
            echo '✓ ROS 2 Humble environment sourced'
        else
            echo '✗ ERROR: ROS 2 setup not found'
            exit 1
        fi
        echo ''

        echo '→ Navigating to workspace...'
        cd /ws
        echo '✓ Working directory: \$(pwd)'
        echo ''

        echo '→ Building turtlebot3_waypoint_navigator package...'
        echo '  Command: colcon build --packages-select turtlebot3_waypoint_navigator'
        echo ''

        colcon build --packages-select turtlebot3_waypoint_navigator

        BUILD_STATUS=\$?
        echo ''

        if [ \$BUILD_STATUS -eq 0 ]; then
            echo '============================================'
            echo '✓ BUILD SUCCESSFUL'
            echo '============================================'
            echo ''

            if [ -f /ws/install/setup.bash ]; then
                echo '✓ Setup files created:'
                echo '  - /ws/install/setup.bash'
                echo '  - /ws/install/setup.sh'
                echo ''
            fi

            echo 'Package built successfully!'
            echo ''
        else
            echo '============================================'
            echo '✗ BUILD FAILED'
            echo '============================================'
            echo ''
            echo 'Build exited with status: '\$BUILD_STATUS
            exit \$BUILD_STATUS
        fi
    "

BUILD_EXIT_CODE=$?

cd "$PROJECT_ROOT"

echo ""

if [ $BUILD_EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}============================================${NC}"
    echo -e "${GREEN}✓ Build Completed Successfully${NC}"
    echo -e "${GREEN}============================================${NC}"
    echo ""
    echo -e "${CYAN}Build Summary:${NC}"
    echo "  Package: turtlebot3_waypoint_navigator"
    echo "  Status: Built and installed"
    echo "  Location: turtlebotrossim/install/"
    echo ""
    echo -e "${CYAN}Files Modified:${NC}"
    echo "  • navigator_nav2.py - Fixed blocking calls"
    echo "  • Replaced time.sleep() with non-blocking timers"
    echo "  • Added timer-based AMCL convergence checking"
    echo ""
    echo -e "${GREEN}Ready to run!${NC}"
    echo ""
    echo -e "${CYAN}Next Steps:${NC}"
    echo ""
    echo "  1. Run the navigator with Docker:"
    echo -e "     ${YELLOW}./docker/run-waypoint-navigator.sh --repetitions 2${NC}"
    echo ""
    echo "  2. Or use docker-compose directly:"
    echo -e "     ${YELLOW}docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator${NC}"
    echo ""
    echo -e "${CYAN}Documentation:${NC}"
    echo "  • FIX_SUMMARY.md - Quick overview of the fix"
    echo "  • WAYPOINT_NAVIGATOR_FIX_README.md - Complete guide"
    echo "  • WAYPOINT_NAVIGATOR_BLOCKING_FIX.md - Technical details"
    echo ""
else
    echo -e "${RED}============================================${NC}"
    echo -e "${RED}✗ Build Failed${NC}"
    echo -e "${RED}============================================${NC}"
    echo ""
    echo "Build exited with code: $BUILD_EXIT_CODE"
    echo ""
    echo -e "${YELLOW}Troubleshooting:${NC}"
    echo "  1. Check for syntax errors in navigator_nav2.py"
    echo "  2. Ensure all dependencies are installed"
    echo "  3. Try a clean rebuild: $0 --clean"
    echo "  4. Check Docker logs above for specific errors"
    echo ""
    exit $BUILD_EXIT_CODE
fi
