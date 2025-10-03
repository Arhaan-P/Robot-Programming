# WanderBot Laser Package

Autonomous wandering robot using laser scanner for obstacle detection.

## Package Contents

- **wander_bot.py**: Autonomous exploration with obstacle avoidance

## Dependencies

```bash
sudo apt-get install ros-noetic-laser-geometry
```

## Building

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/wanderbot_laser/scripts/*.py
```

## Execution

### Option 1: With Gazebo Simulation (Recommended)

#### Terminal 1: Launch Turtlebot3 World
```bash
# Install turtlebot3 simulation
sudo apt-get install ros-noetic-turtlebot3-simulations

# Set robot model
export TURTLEBOT3_MODEL=burger

# Launch world
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

#### Terminal 2: Run WanderBot
```bash
source ~/catkin_ws/devel/setup.bash
rosrun wanderbot_laser wander_bot.py
```

### Option 2: With Custom Gazebo World

#### Terminal 1: Launch Your World
```bash
roslaunch gazebo_ros empty_world.launch
# Spawn robot with laser scanner
```

#### Terminal 2: Run WanderBot
```bash
source ~/catkin_ws/devel/setup.bash
rosrun wanderbot_laser wander_bot.py
```

### Option 3: With Real Robot

```bash
# Connect to robot
source ~/catkin_ws/devel/setup.bash

# Check laser topic
rostopic list | grep scan

# Remap if needed
rosrun wanderbot_laser wander_bot.py /scan:=/your_laser_topic
```

## Expected Output

```
[INFO] WanderBot Started! Exploring...
[WARN] Obstacle detected at 0.65m - Turning!
[WARN] Obstacle detected at 0.52m - Turning!
[WARN] Obstacle detected at 0.71m - Turning!
```

**Behavior:**
- Robot moves forward continuously
- When obstacle detected within 0.8m, robot stops and turns
- Turns in random direction (left or right)
- Continues forward after turning
- Repeats indefinitely

## How It Works

### 1. **LaserScan Subscription**
```python
self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
```

### 2. **Obstacle Detection**
```python
# Get front region (±30 degrees)
front_ranges = msg.ranges[0:30] + msg.ranges[-30:]

# Find minimum distance
min_distance = min(valid_ranges)

# Check threshold
if min_distance < 0.8:
    self.obstacle_detected = True
```

### 3. **Movement Logic**
```python
if obstacle_detected:
    # Turn randomly
    twist.linear.x = 0.0
    twist.angular.z = random.choice([-1, 1]) * 0.5
else:
    # Move forward
    twist.linear.x = 0.3
    twist.angular.z = 0.0
```

## LaserScan Message Structure

```python
# sensor_msgs/LaserScan
float32 angle_min          # Start angle (radians)
float32 angle_max          # End angle (radians)
float32 angle_increment    # Angular distance between measurements
float32 range_min          # Minimum range value (m)
float32 range_max          # Maximum range value (m)
float32[] ranges           # Array of distance measurements
```

**Example:**
- `angle_min = -3.14` (180° to the right)
- `angle_max = 3.14` (180° to the left)
- `ranges[0]` = distance directly ahead
- `ranges[180]` = distance directly behind (if 360° scan)

## Tuning Parameters

Edit values in `wander_bot.py`:

```python
self.obstacle_distance = 0.8   # Detection threshold (meters)
self.linear_speed = 0.3        # Forward speed (m/s)
self.angular_speed = 0.5       # Turning speed (rad/s)
```

**Effect:**
- Increase `obstacle_distance`: Robot turns earlier (more cautious)
- Increase `linear_speed`: Robot moves faster
- Increase `angular_speed`: Robot turns faster

## Advanced Behaviors

### Behavior 1: Wall Following

```python
def laser_callback(self, msg):
    # Get right side readings
    right_ranges = msg.ranges[240:300]  # Right 60°
    right_min = min(r for r in right_ranges if r > msg.range_min)
    
    # Keep wall at 0.5m distance
    desired_distance = 0.5
    error = right_min - desired_distance
    
    twist = Twist()
    twist.linear.x = 0.3
    twist.angular.z = 0.5 * error  # Proportional control
    self.cmd_vel_pub.publish(twist)
```

### Behavior 2: Smart Exploration

```python
import time

class SmartWander:
    def __init__(self):
        self.last_turn_time = time.time()
        self.turn_duration = 0
    
    def laser_callback(self, msg):
        if self.obstacle_detected:
            if self.turn_duration == 0:
                # Start turn
                self.turn_duration = random.uniform(1.0, 3.0)
                self.last_turn_time = time.time()
            
            if time.time() - self.last_turn_time < self.turn_duration:
                # Still turning
                twist.angular.z = 0.5
            else:
                # Done turning
                self.turn_duration = 0
```

### Behavior 3: Follow Corridor

```python
def laser_callback(self, msg):
    left = min(msg.ranges[30:90])
    right = min(msg.ranges[270:330])
    front = min(msg.ranges[0:30] + msg.ranges[-30:])
    
    if front < 0.8:
        # Obstacle ahead - turn
        twist.angular.z = 0.5
    else:
        # Center in corridor
        error = left - right
        twist.linear.x = 0.3
        twist.angular.z = 0.3 * error
```

## Debugging

### Visualize laser scan:
```bash
rosrun rviz rviz
# Add → LaserScan → Topic: /scan
```

### View laser data:
```bash
rostopic echo /scan
```

### Check laser scan rate:
```bash
rostopic hz /scan
```

### Monitor specific ranges:
```bash
rostopic echo /scan/ranges[0]  # Front
```

### Plot laser distances:
```bash
rqt_plot /scan/ranges[0] /scan/ranges[90] /scan/ranges[270]
```

## Troubleshooting

**No laser data:**
```bash
# Check if laser topic exists
rostopic list | grep scan

# Check if laser node is running
rosnode list

# View laser info
rostopic info /scan
```

**Robot doesn't move:**
```bash
# Verify velocity commands
rostopic echo /cmd_vel

# Check if robot is receiving commands
rostopic info /cmd_vel
```

**Constant turning:**
- All front ranges may be invalid (inf/nan)
- Increase `range_max` threshold
- Check laser scanner in Gazebo/RViz

**Crashes into obstacles:**
- Decrease `linear_speed`
- Increase `obstacle_distance`
- Check front region angles match your laser configuration

## Key Concepts for Exam

### LaserScan Basics
- **360° array**: ranges[0] to ranges[359]
- **Index 0**: Straight ahead
- **Index 90**: Left side
- **Index 270**: Right side
- **Invalid readings**: `inf` or values outside [range_min, range_max]

### Obstacle Detection
```python
# Get front readings
front = msg.ranges[0:30]  # Front 30 degrees

# Filter valid
valid = [r for r in front if r > msg.range_min and r < msg.range_max]

# Find closest
min_distance = min(valid)
```

### Random Behavior
```python
import random
direction = random.choice([-1, 1])  # Left or right
```

### State Machine
```
State: FORWARD
  If obstacle detected → TURNING
  
State: TURNING
  If path clear → FORWARD
```

## Testing Without Robot

### Publish fake laser data:
```bash
# Terminal 1
roscore

# Terminal 2
rostopic pub /scan sensor_msgs/LaserScan "header:
  frame_id: 'laser'
ranges: [1.0, 1.0, 0.5, 1.0, 1.0]"

# Terminal 3
rosrun wanderbot_laser wander_bot.py
```

## Real Robot Integration

For real robots (TurtleBot, custom robot):
1. Launch robot driver
2. Verify laser topic: `rostopic list | grep scan`
3. Check frame_id: `rostopic echo /scan | grep frame_id`
4. Run wander_bot.py
5. Monitor in RViz for visualization
