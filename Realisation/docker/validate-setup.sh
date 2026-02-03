#!/bin/bash
#
# Validation Script for TurtleBot 3 Waypoint Navigator Docker Setup
#
# This script checks that all components are properly installed and configured
# Usage: ./docker/validate-setup.sh

set -e

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Counters
PASSED=0
FAILED=0
WARNINGS=0

# Helper functions
print_header() {
    echo -e "\n${BLUE}===================================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}===================================================${NC}\n"
}

check_pass() {
    echo -e "${GREEN}✓ PASS${NC}: $1"
    ((PASSED++))
}

check_fail() {
    echo -e "${RED}✗ FAIL${NC}: $1"
    ((FAILED++))
}

check_warn() {
    echo -e "${YELLOW}⚠ WARN${NC}: $1"
    ((WARNINGS++))
}

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Start validation
print_header "TurtleBot 3 Waypoint Navigator - Setup Validation"

echo "Project Root: $PROJECT_ROOT"
echo "Script Directory: $SCRIPT_DIR"
echo ""

# ============================================================================
# Check 1: Docker Installation
# ============================================================================
print_header "1. Docker Installation"

if command -v docker &> /dev/null; then
    DOCKER_VERSION=$(docker --version)
    check_pass "Docker is installed: $DOCKER_VERSION"
else
    check_fail "Docker is not installed"
fi

if command -v docker &> /dev/null && docker compose version &> /dev/null; then
    COMPOSE_VERSION=$(docker compose version | head -1)
    check_pass "Docker Compose is installed: $COMPOSE_VERSION"
else
    check_fail "Docker Compose is not installed"
fi

# ============================================================================
# Check 2: Project Structure
# ============================================================================
print_header "2. Project Structure"

# Check workspace directory
if [ -d "$PROJECT_ROOT/turtlebotrossim" ]; then
    check_pass "Workspace directory exists: turtlebotrossim/"
else
    check_fail "Workspace directory not found: turtlebotrossim/"
fi

# Check package source
if [ -d "$PROJECT_ROOT/turtlebotrossim/src/turtlebot3_waypoint_navigator" ]; then
    check_pass "Package source exists: turtlebotrossim/src/turtlebot3_waypoint_navigator/"
else
    check_fail "Package source not found: turtlebotrossim/src/turtlebot3_waypoint_navigator/"
fi

# Check docker directory
if [ -d "$PROJECT_ROOT/docker" ]; then
    check_pass "Docker directory exists: docker/"
else
    check_fail "Docker directory not found: docker/"
fi

# ============================================================================
# Check 3: Docker Compose Files
# ============================================================================
print_header "3. Docker Compose Configuration"

COMPOSE_FILE="$PROJECT_ROOT/docker/docker-compose.waypoint-navigator-simple.yaml"
if [ -f "$COMPOSE_FILE" ]; then
    check_pass "Main compose file exists"

    # Validate YAML syntax
    if docker compose -f "$COMPOSE_FILE" config > /dev/null 2>&1; then
        check_pass "Docker Compose file is valid YAML"
    else
        check_fail "Docker Compose file has syntax errors"
    fi
else
    check_fail "Main compose file not found: $COMPOSE_FILE"
fi

# Check base compose file
BASE_COMPOSE="$PROJECT_ROOT/turtlebotrossim/docker/docker-compose.yaml"
if [ -f "$BASE_COMPOSE" ]; then
    check_pass "Base compose file exists: turtlebotrossim/docker/docker-compose.yaml"
else
    check_fail "Base compose file not found: turtlebotrossim/docker/docker-compose.yaml"
fi

# ============================================================================
# Check 4: Helper Scripts
# ============================================================================
print_header "4. Helper Scripts"

HELPER_SCRIPT="$PROJECT_ROOT/docker/run-waypoint-navigator.sh"
if [ -f "$HELPER_SCRIPT" ]; then
    check_pass "Helper script exists: docker/run-waypoint-navigator.sh"

    if [ -x "$HELPER_SCRIPT" ]; then
        check_pass "Helper script is executable"
    else
        check_warn "Helper script is not executable (run: chmod +x $HELPER_SCRIPT)"
    fi
else
    check_fail "Helper script not found: docker/run-waypoint-navigator.sh"
fi

# ============================================================================
# Check 5: Package Files
# ============================================================================
print_header "5. Package Configuration"

PACKAGE_XML="$PROJECT_ROOT/turtlebotrossim/src/turtlebot3_waypoint_navigator/package.xml"
if [ -f "$PACKAGE_XML" ]; then
    check_pass "Package XML exists"
else
    check_fail "Package XML not found"
fi

SETUP_PY="$PROJECT_ROOT/turtlebotrossim/src/turtlebot3_waypoint_navigator/setup.py"
if [ -f "$SETUP_PY" ]; then
    check_pass "setup.py exists"
else
    check_fail "setup.py not found"
fi

NAVIGATOR_TWIST="$PROJECT_ROOT/turtlebotrossim/src/turtlebot3_waypoint_navigator/turtlebot3_waypoint_navigator/navigator_twist.py"
if [ -f "$NAVIGATOR_TWIST" ]; then
    check_pass "navigator_twist.py implementation exists"
else
    check_fail "navigator_twist.py not found"
fi

# ============================================================================
# Check 6: ROS 2 Setup
# ============================================================================
print_header "6. ROS 2 Environment"

if command -v ros2 &> /dev/null; then
    ROS_DISTRO=$(echo $ROS_DISTRO)
    if [ -n "$ROS_DISTRO" ]; then
        check_pass "ROS 2 is installed: $ROS_DISTRO"
    else
        check_warn "ROS 2 is installed but ROS_DISTRO not set (try: source /opt/ros/humble/setup.bash)"
    fi
else
    check_warn "ROS 2 doesn't appear to be sourced (this is OK if Gazebo runs separately)"
fi

if [ -f /opt/ros/humble/setup.bash ]; then
    check_pass "ROS 2 Humble setup.bash found at /opt/ros/humble/setup.bash"
elif [ -f /opt/ros/humble/setup.sh ]; then
    check_pass "ROS 2 Humble setup.sh found at /opt/ros/humble/setup.sh"
else
    check_warn "ROS 2 Humble setup script not found (Docker image may have it)"
fi

# ============================================================================
# Check 7: Workspace Build Status
# ============================================================================
print_header "7. Workspace Build Status"

if [ -f "$PROJECT_ROOT/turtlebotrossim/install/setup.sh" ]; then
    check_pass "Workspace is already built (install/setup.sh exists)"
else
    check_warn "Workspace is not built yet (will be built automatically on first Docker run)"
    echo "  To build manually: cd turtlebotrossim && colcon build --packages-select turtlebot3_waypoint_navigator"
fi

if [ -d "$PROJECT_ROOT/turtlebotrossim/build" ]; then
    check_pass "Build directory exists"
else
    echo "  (Note: build directory will be created on first build)"
fi

# ============================================================================
# Check 8: Docker Image
# ============================================================================
print_header "8. Docker Image"

if command -v docker &> /dev/null; then
    if docker image inspect turtlebot4:devnogpu &> /dev/null; then
        check_pass "turtlebot4:devnogpu image is available locally"
    else
        check_warn "turtlebot4:devnogpu image not found locally (will be built on first docker-compose up)"
    fi
else
    echo "  (Docker not available to check)"
fi

# ============================================================================
# Check 9: Documentation
# ============================================================================
print_header "9. Documentation"

if [ -f "$PROJECT_ROOT/WAYPOINT_DOCKER_SETUP.md" ]; then
    check_pass "Setup documentation exists: WAYPOINT_DOCKER_SETUP.md"
else
    check_warn "Setup documentation not found: WAYPOINT_DOCKER_SETUP.md"
fi

if [ -f "$PROJECT_ROOT/WAYPOINT_DOCKER_COMPLETE.md" ]; then
    check_pass "Completion summary exists: WAYPOINT_DOCKER_COMPLETE.md"
else
    check_warn "Completion summary not found: WAYPOINT_DOCKER_COMPLETE.md"
fi

# ============================================================================
# Check 10: Network and Connectivity
# ============================================================================
print_header "10. Network Setup"

if command -v docker &> /dev/null; then
    # Check if we can reach docker daemon
    if docker ps &> /dev/null; then
        check_pass "Docker daemon is accessible"
    else
        check_fail "Cannot connect to Docker daemon (Is Docker running?)"
    fi
fi

# ============================================================================
# Summary
# ============================================================================
print_header "Validation Summary"

TOTAL=$((PASSED + FAILED + WARNINGS))
echo "Total checks: $TOTAL"
echo -e "${GREEN}Passed: $PASSED${NC}"
echo -e "${RED}Failed: $FAILED${NC}"
echo -e "${YELLOW}Warnings: $WARNINGS${NC}"
echo ""

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}✓ All critical checks passed!${NC}"
    echo ""
    echo "You can now run the waypoint navigator with:"
    echo ""
    echo -e "${BLUE}WAYPOINT_SPEED=0.15 WAYPOINT_POSITION_TOLERANCE=0.05 \\${NC}"
    echo -e "${BLUE}WAYPOINT_ANGLE_TOLERANCE=0.05 WAYPOINT_REPETITIONS=1 \\${NC}"
    echo -e "${BLUE}docker compose -f docker/docker-compose.waypoint-navigator-simple.yaml up waypoint_navigator${NC}"
    echo ""
    echo "Or using the helper script:"
    echo -e "${BLUE}./docker/run-waypoint-navigator.sh --speed 0.15 --position-tolerance 0.05${NC}"
    echo ""
    exit 0
else
    echo -e "${RED}✗ Some critical checks failed!${NC}"
    echo ""
    echo "Please fix the issues listed above before running the navigator."
    echo ""
    exit 1
fi
