# ROS EXAM MASTER EXECUTION GUIDE

**Complete guide for all 6 packages - Ready for your exam!**

---

## 🚀 QUICK START

```bash
# Build everything (do this ONCE)
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## 📦 PACKAGE 1: TURTLE PATTERNS

**Location:** `turtle_patterns/`

### Scripts Available:
1. `move_circle.py` - Circular motion
2. `move_square.py` - Square pattern
3. `move_spiral.py` - Expanding spiral
4. `move_figure8.py` - Figure-8 pattern
5. `move_diamond.py` - Diamond shape
6. `go_to_goal.py` - Navigate to specific (x,y)

### Execution:

```bash
# Terminal 1: ROS Master
roscore

# Terminal 2: Turtlesim
rosrun turtlesim turtlesim_node

# Terminal 3: Pick any pattern
rosrun turtle_patterns move_circle.py
rosrun turtle_patterns move_square.py
rosrun turtle_patterns move_spiral.py
rosrun turtle_patterns move_figure8.py
rosrun turtle_patterns move_diamond.py
rosrun turtle_patterns go_to_goal.py  # Goes to (8,8)
```

### Key Concepts:
- **Topic**: `/turtle1/cmd_vel`
- **Message**: `geometry_msgs/Twist`
- **Pattern**: Publisher only

---

## 📦 PACKAGE 2: TURTLE SERVICES

**Location:** `turtle_services/`

### Services Available:
1. `AddTwoInts` - Simple addition service
2. `GoToPoint` - Navigate turtle to point

### Execution:

#### Test 1: Addition Service
```bash
# Terminal 1
roscore

# Terminal 2: Start server
rosrun turtle_services add_server.py

# Terminal 3: Call client
rosrun turtle_services add_client.py
```

**Expected Output:**
```
10 + 15 = 25
50 + 100 = 150
```

#### Test 2: Turtle Navigation Service
```bash
# Terminal 1
roscore

# Terminal 2: Turtlesim
rosrun turtlesim turtlesim_node

# Terminal 3: Start server
rosrun turtle_services goto_server.py

# Terminal 4: Call client
rosrun turtle_services goto_client.py
```

### Key Concepts:
- **Pattern**: Request → Response
- **Synchronous**: Blocks until complete
- **Command line test**:
  ```bash
  rosservice call /add "a: 10
  b: 20"
  ```

---

## 📦 PACKAGE 3: TURTLE ACTIONS

**Location:** `turtle_actions/`

### Action Available:
- `NavigateToGoal` - Long-running navigation with feedback

### Execution:

```bash
# Terminal 1
roscore

# Terminal 2: Turtlesim
rosrun turtlesim turtlesim_node

# Terminal 3: Action Server
rosrun turtle_actions navigate_server.py

# Terminal 4: Action Client
rosrun turtle_actions navigate_client.py
```

**Expected Output:**
```
[INFO] Distance remaining: 6.32m, Progress: 12.5%
[INFO] Distance remaining: 4.21m, Progress: 35.2%
[INFO] Distance remaining: 2.15m, Progress: 68.9%
[INFO] SUCCESS! Final distance: 0.0823m
```

### Key Concepts:
- **5 Topics**: goal, cancel, status, feedback, result
- **Asynchronous**: Non-blocking
- **Feedback**: Real-time progress updates
- **Cancellable**: Can preempt goals

---

## 📦 PACKAGE 4: LINE FOLLOWER CAMERA

**Location:** `line_follower_camera/`

### Script Available:
- `line_follower.py` - OpenCV-based line follower

### Execution (Simulation):

```bash
# Install dependencies first
sudo apt-get install ros-noetic-cv-bridge python3-opencv

# Terminal 1
roscore

# Terminal 2: Launch Gazebo with robot + camera
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Terminal 3: Run line follower
rosrun line_follower_camera line_follower.py
```

### Key Concepts:
- **Image Processing**:
  1. Subscribe to `/camera/rgb/image_raw`
  2. Convert ROS Image → OpenCV with `cv_bridge`
  3. BGR → HSV color space
  4. Create mask with `cv2.inRange()`
  5. Find contours with `cv2.findContours()`
  6. Calculate centroid with `cv2.moments()`
  7. Proportional control

- **Control Logic**:
  ```python
  error = line_center - image_center
  angular_velocity = -K_p * error
  ```

---

## 📦 PACKAGE 5: WANDERBOT LASER

**Location:** `wanderbot_laser/`

### Script Available:
- `wander_bot.py` - Autonomous exploration

### Execution:

```bash
# Install TurtleBot3 simulation
sudo apt-get install ros-noetic-turtlebot3-simulations

# Terminal 1: Launch world
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

# Terminal 2: Run wanderbot
rosrun wanderbot_laser wander_bot.py
```

**Expected Behavior:**
- Moves forward continuously
- Detects obstacles within 0.8m
- Turns randomly when obstacle detected
- Continues exploring

### Key Concepts:
- **LaserScan Message**:
  - `ranges[]` - Array of distances
  - `ranges[0]` - Front
  - `ranges[90]` - Left
  - `ranges[270]` - Right

- **Obstacle Detection**:
  ```python
  front_ranges = msg.ranges[0:30] + msg.ranges[-30:]
  min_distance = min(front_ranges)
  if min_distance < 0.8:
      turn()
  ```

---

## 📦 PACKAGE 6: OBSTACLE AVOIDANCE ULTRASONIC

**Location:** `obstacle_avoidance_ultrasonic/`

### Scripts Available:
- `obstacle_avoider.py` - Obstacle avoidance logic
- `ultrasonic_simulator.py` - Test simulator

### Execution (With Simulator):

```bash
# Terminal 1
roscore

# Terminal 2: Simulator
rosrun obstacle_avoidance_ultrasonic ultrasonic_simulator.py

# Terminal 3: Avoider
rosrun obstacle_avoidance_ultrasonic obstacle_avoider.py
```

**Expected Output:**
```
[INFO] Distance: 0.85m
[INFO] Distance: 0.70m
[WARN] OBSTACLE at 0.45m - Turning!
```

### Key Concepts:
- **Range Message**:
  - Single distance value
  - `msg.range` - Distance in meters
  - `msg.min_range` / `msg.max_range` - Valid range

- **Simple Logic**:
  ```python
  if distance < threshold:
      turn_right()
  else:
      go_forward()
  ```

---

## 🔍 DEBUGGING COMMANDS

### General
```bash
# List all nodes
rosnode list

# List all topics
rostopic list

# Echo topic data
rostopic echo /topic_name

# Topic info
rostopic info /topic_name

# Message type
rostopic type /topic_name

# Message definition
rosmsg show MessageType
```

### Services
```bash
# List services
rosservice list

# Service info
rosservice info /service_name

# Service type
rosservice type /service_name

# Call service
rosservice call /service_name "args"
```

### Actions
```bash
# List action topics
rostopic list | grep action_name

# Monitor feedback
rostopic echo /action_name/feedback

# Monitor status
rostopic echo /action_name/status
```

---

## 🎯 EXAM QUICK REFERENCE

### When to use what?

| Communication | Use Case | Example |
|--------------|----------|---------|
| **Topic** | Continuous data stream | Sensor readings, robot velocity |
| **Service** | Request-response, quick tasks | "Add two numbers", "Get position" |
| **Action** | Long tasks with feedback | "Navigate to goal", "Pick object" |

### Publisher-Subscriber Pattern
```python
# Publisher
pub = rospy.Publisher('/topic', MessageType, queue_size=10)
pub.publish(msg)

# Subscriber
sub = rospy.Subscriber('/topic', MessageType, callback)
```

### Service Pattern
```python
# Server
service = rospy.Service('/service', ServiceType, handle_request)

# Client
rospy.wait_for_service('/service')
client = rospy.ServiceProxy('/service', ServiceType)
response = client(request)
```

### Action Pattern
```python
# Server
server = actionlib.SimpleActionServer('action', ActionType, execute, False)
server.publish_feedback(feedback)
server.set_succeeded(result)

# Client
client = actionlib.SimpleActionClient('action', ActionType)
client.wait_for_server()
client.send_goal(goal, feedback_cb=callback)
result = client.get_result()
```

---

## ⚠️ TROUBLESHOOTING

### "No module named 'package_name.msg'"
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### "Permission denied" when running script
```bash
chmod +x ~/catkin_ws/src/package_name/scripts/script_name.py
```

### "Unable to contact my own server"
```bash
# Start roscore first
roscore
```

### Node not publishing/subscribing
```bash
# Check if topic exists
rostopic list

# Check if node is running
rosnode list

# View topic data
rostopic echo /topic_name
```

### Service not found
```bash
# List services
rosservice list

# Check if server node is running
rosnode list
```

### Action server not available
```bash
# Check action topics
rostopic list | grep action_name

# Verify server node running
rosnode list
```

---

## 📝 BEFORE YOUR EXAM

### Checklist:
- [ ] Build all packages: `cd ~/catkin_ws && catkin_make`
- [ ] Source workspace: `source ~/catkin_ws/devel/setup.bash`
- [ ] Test each package at least once
- [ ] Review individual package READMEs
- [ ] Memorize communication patterns (pub/sub, service, action)
- [ ] Practice debugging commands

### Add to ~/.bashrc (optional but helpful):
```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## 🎓 EXAM TIPS

1. **Always start roscore first** (Terminal 1)
2. **Source workspace in every terminal**: `source ~/catkin_ws/devel/setup.bash`
3. **Check topics/nodes if something doesn't work**: `rostopic list`, `rosnode list`
4. **Read error messages carefully** - they usually tell you what's wrong
5. **Remember message types**:
   - Twist → velocity
   - Image → camera
   - LaserScan → laser
   - Range → ultrasonic

6. **Build order matters**: Definition files (.srv, .action) → catkin_make → Scripts

---

## 📚 ADDITIONAL RESOURCES

- **ROS_CODING_EXAM_GUIDE.md** - Detailed code templates ⭐
- **SERVICE_ACTION_CONFIGURATION_GUIDE.md** - CMakeLists.txt & package.xml setup for custom messages 🔴
- **ROS_CHEAT_SHEET.md** - 1-page quick reference
- **Package READMEs** - Detailed docs for each package

---

## ✅ FINAL CHECK

Run this to verify everything works:

```bash
# Test build
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# Test simple pattern
roscore &
sleep 2
rosrun turtlesim turtlesim_node &
sleep 2
rosrun turtle_patterns move_circle.py

# If turtle draws a circle, you're ready! ✅
```

---

## 🆘 EMERGENCY REBUILD

If anything goes wrong:

```bash
cd ~/catkin_ws
rm -rf build/ devel/
catkin_make
source devel/setup.bash
chmod +x src/*/scripts/*.py
```

---

**Good luck on your exam! You've got this! 🚀**

**All packages are working and ready to run. Just follow the commands above!**
