# Obstacle Avoidance Ultrasonic Package

Simple obstacle avoidance using ultrasonic/range sensor.

## Package Contents

- **obstacle_avoider.py**: Obstacle avoidance controller
- **ultrasonic_simulator.py**: Simulated ultrasonic sensor (for testing)

## Dependencies

No special dependencies required - uses standard ROS messages.

## Building

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/obstacle_avoidance_ultrasonic/scripts/*.py
```

## Execution

### Option 1: With Simulator (For Testing Without Robot)

#### Terminal 1: Start ROS Master
```bash
roscore
```

#### Terminal 2: Run Ultrasonic Simulator
```bash
source ~/catkin_ws/devel/setup.bash
rosrun obstacle_avoidance_ultrasonic ultrasonic_simulator.py
```

You should see:
```
[INFO] Ultrasonic Simulator Started!
[INFO] Distance: 2.00m
[INFO] Distance: 1.95m
[INFO] Distance: 1.90m
```

#### Terminal 3: Run Obstacle Avoider
```bash
source ~/catkin_ws/devel/setup.bash
rosrun obstacle_avoidance_ultrasonic obstacle_avoider.py
```

You should see:
```
[INFO] Obstacle Avoider Started!
[INFO] Distance: 0.85m
[INFO] Distance: 0.70m
[WARN] OBSTACLE at 0.45m - Turning!
[WARN] OBSTACLE at 0.30m - Turning!
```

### Option 2: With Gazebo Robot

#### Terminal 1: Launch Gazebo with Robot
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

#### Terminal 2: Run Obstacle Avoider (with topic remapping)
```bash
source ~/catkin_ws/devel/setup.bash
# Check your ultrasonic topic name first
rostopic list | grep range

# Run with remapping if needed
rosrun obstacle_avoidance_ultrasonic obstacle_avoider.py /ultrasonic:=/your_sensor_topic
```

### Option 3: With Real Robot

#### Terminal 1: Connect to Robot
```bash
# SSH to robot or launch robot driver
roslaunch my_robot robot.launch
```

#### Terminal 2: Run Obstacle Avoider
```bash
source ~/catkin_ws/devel/setup.bash
rosrun obstacle_avoidance_ultrasonic obstacle_avoider.py
```

## Expected Behavior

**When path is clear (distance > 0.5m):**
- Robot moves forward at 0.3 m/s
- No turning

**When obstacle detected (distance < 0.5m):**
- Robot stops moving forward
- Robot turns right at 0.8 rad/s
- Continues turning until path clears

## How It Works

### 1. **Range Sensor Subscription**
```python
self.ultrasonic_sub = rospy.Subscriber('/ultrasonic', Range, self.ultrasonic_callback)
```

### 2. **Distance Measurement**
```python
def ultrasonic_callback(self, msg):
    self.current_distance = msg.range
    
    # Validate reading
    if msg.range < msg.min_range or msg.range > msg.max_range:
        return  # Invalid
```

### 3. **Avoidance Logic**
```python
if self.current_distance < self.safe_distance:
    # Turn right
    twist.linear.x = 0.0
    twist.angular.z = -0.8
else:
    # Go straight
    twist.linear.x = 0.3
    twist.angular.z = 0.0
```

## Range Message Structure

```python
# sensor_msgs/Range
Header header
uint8 radiation_type    # ULTRASOUND=0, INFRARED=1
float32 field_of_view   # Arc angle (radians)
float32 min_range       # Minimum distance (meters)
float32 max_range       # Maximum distance (meters)
float32 range           # Measured distance (meters)
```

**Radiation Types:**
- `Range.ULTRASOUND = 0` (HC-SR04, MaxBotix)
- `Range.INFRARED = 1` (Sharp GP2Y0A21YK)

## Tuning Parameters

Edit values in `obstacle_avoider.py`:

```python
self.safe_distance = 0.5    # Detection threshold (meters)
self.linear_speed = 0.3     # Forward speed (m/s)
self.angular_speed = 0.8    # Turning speed (rad/s)
```

**Effects:**
- Increase `safe_distance`: More cautious (turns earlier)
- Increase `linear_speed`: Faster movement
- Increase `angular_speed`: Faster turning

## Advanced Behaviors

### Behavior 1: Turn Left Instead
```python
if self.current_distance < self.safe_distance:
    twist.angular.z = +self.angular_speed  # Positive = left
```

### Behavior 2: Turn Random Direction
```python
import random

if self.current_distance < self.safe_distance:
    if not hasattr(self, 'turn_direction'):
        self.turn_direction = random.choice([-1, 1])
    twist.angular.z = self.turn_direction * self.angular_speed
else:
    if hasattr(self, 'turn_direction'):
        del self.turn_direction  # Reset for next obstacle
```

### Behavior 3: Gradual Slowdown
```python
def avoid_obstacle(self):
    if self.current_distance < self.safe_distance:
        # Stop and turn
        twist.linear.x = 0.0
        twist.angular.z = -0.8
    elif self.current_distance < 1.0:
        # Slow down proportionally
        speed_ratio = (self.current_distance - self.safe_distance) / 0.5
        twist.linear.x = self.linear_speed * speed_ratio
        twist.angular.z = 0.0
    else:
        # Full speed
        twist.linear.x = self.linear_speed
        twist.angular.z = 0.0
```

### Behavior 4: Multiple Sensors
```python
class MultiSensorAvoider:
    def __init__(self):
        # Subscribe to multiple sensors
        rospy.Subscriber('/ultrasonic_front', Range, self.front_callback)
        rospy.Subscriber('/ultrasonic_left', Range, self.left_callback)
        rospy.Subscriber('/ultrasonic_right', Range, self.right_callback)
        
        self.front_dist = float('inf')
        self.left_dist = float('inf')
        self.right_dist = float('inf')
    
    def avoid_obstacle(self):
        if self.front_dist < 0.5:
            # Obstacle in front
            if self.left_dist > self.right_dist:
                twist.angular.z = 0.8  # Turn left
            else:
                twist.angular.z = -0.8  # Turn right
        else:
            twist.linear.x = 0.3
```

## Debugging

### View ultrasonic data:
```bash
rostopic echo /ultrasonic
```

### Check message rate:
```bash
rostopic hz /ultrasonic
```

### Monitor specific distance:
```bash
rostopic echo /ultrasonic/range
```

### Plot distance over time:
```bash
rqt_plot /ultrasonic/range
```

### Check valid range:
```bash
rostopic echo /ultrasonic | grep -E "min_range|max_range"
```

### Visualize in RViz:
```bash
rosrun rviz rviz
# Add → Range → Topic: /ultrasonic
```

## Troubleshooting

**No ultrasonic data:**
```bash
# Check if topic exists
rostopic list | grep ultrasonic

# Check message type
rostopic type /ultrasonic

# Verify it's sensor_msgs/Range
```

**Robot doesn't avoid obstacles:**
- Check `safe_distance` is appropriate for your sensor
- Verify sensor is publishing valid data: `rostopic echo /ultrasonic`
- Ensure robot can receive `/cmd_vel` commands

**Constant turning:**
- Sensor may be reading invalid data (inf/nan)
- Add validation: `if msg.range < msg.min_range: return`
- Check sensor mounting and wiring

**Robot moves but doesn't turn:**
- Verify `/cmd_vel` is correct topic for your robot
- Check robot's maximum angular velocity
- Increase `angular_speed`

## Key Concepts for Exam

### Range vs LaserScan
- **Range**: Single distance measurement (1 value)
- **LaserScan**: Multiple distance measurements (array of values)

### Ultrasonic Characteristics
- **Range**: Typically 0.02m to 4m
- **Field of View**: ~15-30 degrees
- **Update Rate**: 10-40 Hz
- **Radiation Type**: ULTRASOUND (0)

### Simple Avoidance Logic
```python
if distance < threshold:
    turn()
else:
    go_forward()
```

### State Machine
```
State: FORWARD
  If obstacle → TURNING

State: TURNING
  If clear → FORWARD
```

## Hardware Integration

### HC-SR04 Ultrasonic Sensor (Arduino)

**Arduino Code:**
```cpp
#include <ros.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;
sensor_msgs::Range range_msg;
ros::Publisher pub_range("/ultrasonic", &range_msg);

const int TRIG = 9;
const int ECHO = 10;

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  
  nh.initNode();
  nh.advertise(pub_range);
  
  range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_msg.header.frame_id = "ultrasonic";
  range_msg.field_of_view = 0.26;  // ~15 degrees
  range_msg.min_range = 0.02;
  range_msg.max_range = 4.0;
}

void loop() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  
  long duration = pulseIn(ECHO, HIGH);
  float distance = duration * 0.034 / 2 / 100;  // Convert to meters
  
  range_msg.range = distance;
  range_msg.header.stamp = nh.now();
  pub_range.publish(&range_msg);
  
  nh.spinOnce();
  delay(100);
}
```

### Raspberry Pi GPIO (Python)

**Python Code:**
```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
import RPi.GPIO as GPIO
import time

TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

def get_distance():
    GPIO.output(TRIG, False)
    time.sleep(0.00001)
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    
    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  # Speed of sound
    return distance / 100  # Convert to meters

rospy.init_node('ultrasonic_sensor')
pub = rospy.Publisher('/ultrasonic', Range, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    msg = Range()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "ultrasonic"
    msg.radiation_type = Range.ULTRASOUND
    msg.field_of_view = 0.26
    msg.min_range = 0.02
    msg.max_range = 4.0
    msg.range = get_distance()
    
    pub.publish(msg)
    rate.sleep()
```

## Testing Without Hardware

Use the included simulator:
```bash
# Terminal 1
roscore

# Terminal 2 - Simulator
rosrun obstacle_avoidance_ultrasonic ultrasonic_simulator.py

# Terminal 3 - Avoider
rosrun obstacle_avoidance_ultrasonic obstacle_avoider.py

# Terminal 4 - Monitor
rostopic echo /ultrasonic
```

Or publish manually:
```bash
rostopic pub /ultrasonic sensor_msgs/Range "header:
  frame_id: 'ultrasonic'
radiation_type: 0
field_of_view: 0.1
min_range: 0.02
max_range: 4.0
range: 0.3"
```
