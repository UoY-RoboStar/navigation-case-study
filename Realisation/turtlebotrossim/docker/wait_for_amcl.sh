#!/bin/bash
################################################################################
# Wait for AMCL and Navigation2 to be Ready
#
# This script waits for all necessary ROS2 nodes and topics to be available
# before starting the waypoint navigator. This ensures the initial pose can
# be properly received by AMCL.
#
# Checks performed:
# 1. ROS2 environment is sourced
# 2. ROS2 daemon is responsive
# 3. Critical topics are being published (/map, /scan, /clock)
# 4. AMCL node is running
# 5. AMCL is subscribing to /initialpose
# 6. Nav2 action servers are available
################################################################################

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Configuration
MAX_WAIT_TIME=180  # Maximum wait time in seconds (3 minutes)
CHECK_INTERVAL=2   # Check every 2 seconds

echo -e "${BLUE}=================================${NC}"
echo -e "${BLUE}AMCL Readiness Checker${NC}"
echo -e "${BLUE}=================================${NC}"
echo ""

# Source ROS2 environment
echo -e "${CYAN}[1/6]${NC} Sourcing ROS2 environment..."
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
echo -e "${GREEN}✓${NC} ROS2 environment sourced"
echo ""

# Function to check if a topic exists and has publishers
check_topic_publishers() {
    local topic=$1
    local topic_info=$(ros2 topic info "$topic" 2>/dev/null || echo "")
    if [[ -z "$topic_info" ]]; then
        return 1
    fi
    # Check if there's at least one publisher
    local pub_count=$(echo "$topic_info" | grep -c "Publisher count:" || echo "0")
    if [[ $pub_count -eq 0 ]]; then
        return 1
    fi
    return 0
}

# Function to check if a topic has subscribers
check_topic_subscribers() {
    local topic=$1
    local min_subscribers=${2:-1}
    local sub_count=$(ros2 topic info "$topic" 2>/dev/null | grep "Subscription count:" | awk '{print $3}' || echo "0")
    if [[ $sub_count -ge $min_subscribers ]]; then
        return 0
    fi
    return 1
}

# Function to check if a node is running
check_node_running() {
    local node_pattern=$1
    ros2 node list 2>/dev/null | grep -q "$node_pattern"
    return $?
}

# Function to check if action server is available
check_action_server() {
    local action_name=$1
    timeout 5 ros2 action list 2>/dev/null | grep -q "$action_name"
    return $?
}

start_time=$(date +%s)

# Check 1: Wait for ROS2 topics to be available
echo -e "${CYAN}[2/6]${NC} Waiting for ROS2 daemon to be responsive..."
attempt=0
max_attempts=$((MAX_WAIT_TIME / CHECK_INTERVAL))

while [ $attempt -lt $max_attempts ]; do
    if ros2 topic list &>/dev/null; then
        echo -e "${GREEN}✓${NC} ROS2 daemon is responsive"
        break
    fi
    attempt=$((attempt + 1))
    elapsed=$((attempt * CHECK_INTERVAL))
    echo -e "${YELLOW}⏳${NC} Still waiting... (${elapsed}s elapsed)"
    sleep $CHECK_INTERVAL
done

if [ $attempt -ge $max_attempts ]; then
    echo -e "${RED}✗${NC} Timeout: ROS2 daemon did not become responsive"
    exit 1
fi
echo ""

# Check 2: Wait for critical topics to be published
echo -e "${CYAN}[3/6]${NC} Waiting for critical topics..."
critical_topics=("/map" "/scan" "/clock")
attempt=0

while [ $attempt -lt $max_attempts ]; do
    all_topics_ready=true

    for topic in "${critical_topics[@]}"; do
        if ! check_topic_publishers "$topic"; then
            all_topics_ready=false
            elapsed=$((attempt * CHECK_INTERVAL))
            echo -e "${YELLOW}⏳${NC} Waiting for ${topic}... (${elapsed}s elapsed)"
            break
        fi
    done

    if [ "$all_topics_ready" = true ]; then
        echo -e "${GREEN}✓${NC} All critical topics are being published:"
        for topic in "${critical_topics[@]}"; do
            echo -e "  ${GREEN}✓${NC} $topic"
        done
        break
    fi

    attempt=$((attempt + 1))
    sleep $CHECK_INTERVAL
done

if [ $attempt -ge $max_attempts ]; then
    echo -e "${RED}✗${NC} Timeout: Not all critical topics became available"
    exit 1
fi
echo ""

# Check 3: Wait for AMCL node to be running
echo -e "${CYAN}[4/6]${NC} Waiting for AMCL node..."
attempt=0

while [ $attempt -lt $max_attempts ]; do
    if check_node_running "amcl"; then
        echo -e "${GREEN}✓${NC} AMCL node is running"
        break
    fi
    attempt=$((attempt + 1))
    elapsed=$((attempt * CHECK_INTERVAL))
    echo -e "${YELLOW}⏳${NC} Waiting for AMCL... (${elapsed}s elapsed)"
    sleep $CHECK_INTERVAL
done

if [ $attempt -ge $max_attempts ]; then
    echo -e "${RED}✗${NC} Timeout: AMCL node did not start"
    exit 1
fi
echo ""

# Check 4: Wait for AMCL to subscribe to /initialpose
echo -e "${CYAN}[5/6]${NC} Waiting for AMCL to subscribe to /initialpose..."
attempt=0

while [ $attempt -lt $max_attempts ]; do
    if check_topic_subscribers "/initialpose" 1; then
        sub_count=$(ros2 topic info "/initialpose" 2>/dev/null | grep "Subscription count:" | awk '{print $3}')
        echo -e "${GREEN}✓${NC} AMCL is subscribed to /initialpose ($sub_count subscriber(s))"
        break
    fi
    attempt=$((attempt + 1))
    elapsed=$((attempt * CHECK_INTERVAL))
    echo -e "${YELLOW}⏳${NC} Waiting for AMCL to subscribe... (${elapsed}s elapsed)"
    sleep $CHECK_INTERVAL
done

if [ $attempt -ge $max_attempts ]; then
    echo -e "${RED}✗${NC} Timeout: AMCL did not subscribe to /initialpose"
    echo -e "${YELLOW}⚠${NC}  Continuing anyway, but initial pose may not be received properly"
fi
echo ""

# Check 5: Wait for Nav2 action servers
echo -e "${CYAN}[6/6]${NC} Waiting for Navigation2 action servers..."
attempt=0
action_servers=("navigate_to_pose")

while [ $attempt -lt $max_attempts ]; do
    all_servers_ready=true

    for server in "${action_servers[@]}"; do
        if ! check_action_server "$server"; then
            all_servers_ready=false
            elapsed=$((attempt * CHECK_INTERVAL))
            echo -e "${YELLOW}⏳${NC} Waiting for /${server} action server... (${elapsed}s elapsed)"
            break
        fi
    done

    if [ "$all_servers_ready" = true ]; then
        echo -e "${GREEN}✓${NC} All Nav2 action servers are available:"
        for server in "${action_servers[@]}"; do
            echo -e "  ${GREEN}✓${NC} /${server}"
        done
        break
    fi

    attempt=$((attempt + 1))
    sleep $CHECK_INTERVAL
done

if [ $attempt -ge $max_attempts ]; then
    echo -e "${RED}✗${NC} Timeout: Not all Nav2 action servers became available"
    exit 1
fi
echo ""

# All checks passed
end_time=$(date +%s)
total_time=$((end_time - start_time))

echo -e "${GREEN}=================================${NC}"
echo -e "${GREEN}✓ All systems ready!${NC}"
echo -e "${GREEN}=================================${NC}"
echo -e "Total wait time: ${total_time}s"
echo ""

# Additional stabilization delay
echo -e "${CYAN}Waiting additional 5 seconds for system stabilization...${NC}"
sleep 5

echo -e "${BLUE}Starting Waypoint Navigator...${NC}"
echo ""

# Start the waypoint navigator
# Call the executable directly instead of using ros2 run
# (ros2 run has issues finding the executable in this setup)
exec /ws/install/turtlebot3_waypoint_navigator/bin/waypoint_navigator_nav2
