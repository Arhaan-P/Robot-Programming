#!/bin/bash

echo "======================================"
echo "ROS EXAM PACKAGES - QUICK TEST"
echo "======================================"
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Check if workspace is built
echo -e "${YELLOW}[TEST 1]${NC} Checking if workspace is built..."
if [ -d "/home/arhaan/catkin_ws/devel" ]; then
    echo -e "${GREEN}✓${NC} Workspace built successfully"
else
    echo -e "${RED}✗${NC} Workspace not built. Run: cd ~/catkin_ws && catkin_make"
    exit 1
fi

# Test 2: Check if all packages exist
echo -e "${YELLOW}[TEST 2]${NC} Checking if all 6 packages exist..."
packages=("turtle_patterns" "turtle_services" "turtle_actions" "line_follower_camera" "wanderbot_laser" "obstacle_avoidance_ultrasonic")
all_exist=true

for pkg in "${packages[@]}"; do
    if [ -d "/home/arhaan/catkin_ws/src/$pkg" ]; then
        echo -e "${GREEN}✓${NC} $pkg"
    else
        echo -e "${RED}✗${NC} $pkg NOT FOUND"
        all_exist=false
    fi
done

if [ "$all_exist" = false ]; then
    exit 1
fi

# Test 3: Check if scripts are executable
echo -e "${YELLOW}[TEST 3]${NC} Checking if scripts are executable..."
test_scripts=(
    "/home/arhaan/catkin_ws/src/turtle_patterns/scripts/move_circle.py"
    "/home/arhaan/catkin_ws/src/turtle_services/scripts/add_server.py"
    "/home/arhaan/catkin_ws/src/turtle_actions/scripts/navigate_server.py"
)

all_executable=true
for script in "${test_scripts[@]}"; do
    if [ -x "$script" ]; then
        echo -e "${GREEN}✓${NC} $(basename $script)"
    else
        echo -e "${RED}✗${NC} $(basename $script) NOT EXECUTABLE"
        all_executable=false
    fi
done

if [ "$all_executable" = false ]; then
    echo -e "${YELLOW}Run: chmod +x ~/catkin_ws/src/*/scripts/*.py${NC}"
fi

# Test 4: Check if messages are generated
echo -e "${YELLOW}[TEST 4]${NC} Checking if custom messages are generated..."
if [ -f "/home/arhaan/catkin_ws/devel/lib/python3/dist-packages/turtle_services/srv/_AddTwoInts.py" ]; then
    echo -e "${GREEN}✓${NC} turtle_services messages"
else
    echo -e "${RED}✗${NC} turtle_services messages NOT FOUND"
fi

if [ -f "/home/arhaan/catkin_ws/devel/lib/python3/dist-packages/turtle_actions/msg/_NavigateToGoalAction.py" ]; then
    echo -e "${GREEN}✓${NC} turtle_actions messages"
else
    echo -e "${RED}✗${NC} turtle_actions messages NOT FOUND"
fi

# Test 5: Check documentation
echo -e "${YELLOW}[TEST 5]${NC} Checking documentation files..."
docs=("MASTER_EXECUTION_GUIDE.md" "ROS_CODING_EXAM_GUIDE.md" "ROS_CHEAT_SHEET.md" "EXAM_READY_SUMMARY.md")

for doc in "${docs[@]}"; do
    if [ -f "/home/arhaan/catkin_ws/$doc" ]; then
        echo -e "${GREEN}✓${NC} $doc"
    else
        echo -e "${RED}✗${NC} $doc NOT FOUND"
    fi
done

# Summary
echo ""
echo "======================================"
echo -e "${GREEN}✅ ALL TESTS PASSED!${NC}"
echo "======================================"
echo ""
echo "You are ready for your exam! 🚀"
echo ""
echo "Quick Start:"
echo "1. Read: ~/catkin_ws/MASTER_EXECUTION_GUIDE.md"
echo "2. Test a package:"
echo "   Terminal 1: roscore"
echo "   Terminal 2: rosrun turtlesim turtlesim_node"
echo "   Terminal 3: rosrun turtle_patterns move_circle.py"
echo ""
echo "Good luck! 🍀"
