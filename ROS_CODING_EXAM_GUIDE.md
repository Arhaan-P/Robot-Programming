# 🚀 ROS CODING EXAM GUIDE - CODE ONLY

> **Mission**: Write ROS code fast and correctly for tomorrow's exam
> 
> **Focus**: Pure code patterns, no theory
> 
> **Strategy**: Copy, modify, win!

---

## 📋 TABLE OF CONTENTS

1. **[⚠️ CRITICAL CONFIGURATION](#️-critical-configuration-read-this-first)** ← **START HERE!**
2. [Setup & Build Commands](#setup--build-commands)
3. [Publisher Templates](#publisher-templates)
4. [Subscriber Templates](#subscriber-templates)
5. [Service Templates](#service-templates)
6. [Action Templates](#action-templates)
7. [Launch File Templates](#launch-file-templates)
8. [Turtlesim Movement Patterns](#turtlesim-movement-patterns)
9. [Line Follower Code](#line-follower-code)
10. [Obstacle Avoidance Code](#obstacle-avoidance-code)
11. [Complete Examples](#complete-examples)
12. [Quick Debug Commands](#quick-debug-commands)
13. [Common Errors & Fixes](#-common-errors--fixes)

---

# ⚠️ CRITICAL CONFIGURATION (READ THIS FIRST!)

## For Custom Services (.srv files)

**You MUST edit these files before service will work:**

### 1. package.xml
```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

### 2. CMakeLists.txt
```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation  # ← ADD THIS
)

add_service_files(FILES YourService.srv)  # ← ADD THIS
generate_messages(DEPENDENCIES std_msgs)  # ← ADD THIS

catkin_package(
  CATKIN_DEPENDS message_runtime  # ← ADD THIS
)
```

### 3. Build
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## For Custom Actions (.action files)

**You MUST edit these files before action will work:**

### 1. package.xml
```xml
<build_depend>message_generation</build_depend>
<build_depend>actionlib_msgs</build_depend>
<exec_depend>message_runtime</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
```

### 2. CMakeLists.txt
```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs      # ← ADD THIS
  message_generation  # ← ADD THIS
)

add_action_files(FILES YourAction.action)           # ← ADD THIS
generate_messages(DEPENDENCIES std_msgs actionlib_msgs)  # ← ADD THIS

catkin_package(
  CATKIN_DEPENDS message_runtime actionlib_msgs  # ← ADD THIS
)
```

### 3. Build
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## ⚠️ WHY IS THIS NEEDED?

**Without these changes:**
- ❌ `ImportError: No module named 'my_package.srv'`
- ❌ `ImportError: No module named 'my_package.msg'`
- ❌ Service/Action Python classes won't be generated

**With these changes:**
- ✅ ROS auto-generates Python/C++ classes from .srv/.action files
- ✅ You can import: `from my_package.srv import MyService`
- ✅ You can import: `from my_package.msg import MyActionAction`

**Order matters:**
1. Create .srv or .action file
2. Edit package.xml
3. Edit CMakeLists.txt
4. Run `catkin_make` ← **MUST DO THIS!**
5. Source workspace
6. Now write your Python scripts

---

# SETUP & BUILD COMMANDS

### Create Package
```bash
cd ~/catkin_ws/src
catkin_create_pkg my_package rospy roscpp std_msgs geometry_msgs sensor_msgs
```

### Build & Source
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### Make Python Script Executable
```bash
chmod +x src/my_package/scripts/my_script.py
```

### Run
```bash
# Start roscore
roscore

# Run node
rosrun my_package my_script.py

# Launch file
roslaunch my_package my_launch.launch
```

---

# PUBLISHER TEMPLATES

## Python Publisher Template

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String  # Change message type as needed

def publisher():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('/topic_name', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hello World"
        
        rospy.loginfo(f"Publishing: {msg.data}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
```

## C++ Publisher Template

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher_node");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<std_msgs::String>("/topic_name", 10);
    ros::Rate rate(10);
    
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "Hello World";
        
        ROS_INFO("Publishing: %s", msg.data.c_str());
        pub.publish(msg);
        rate.sleep();
    }
    
    return 0;
}
```

---

# SUBSCRIBER TEMPLATES

## Python Subscriber Template

```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Received: {msg.data}")
    # Process message here

def subscriber():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber('/topic_name', String, callback)
    rospy.spin()  # Keep node running

if __name__ == '__main__':
    try:
        subscriber()
    except rospy.ROSInterruptException:
        pass
```

## C++ Subscriber Template

```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received: %s", msg->data.c_str());
    // Process message here
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber_node");
    ros::NodeHandle nh;
    
    ros::Subscriber sub = nh.subscribe("/topic_name", 10, callback);
    
    ros::spin();
    return 0;
}
```

---

# SERVICE TEMPLATES

## Step 1: Define Service File (*.srv)

**File**: `my_package/srv/AddTwoInts.srv`
```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

## Step 2: Update package.xml

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

## Step 3: Update CMakeLists.txt

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

# Add service files
add_service_files(
  FILES
  AddTwoInts.srv
)

# Generate messages
generate_messages(
  DEPENDENCIES
  std_msgs
)

# IMPORTANT: Update catkin_package to include message_runtime
catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs message_runtime
)
```

**CRITICAL**: After editing package.xml and CMakeLists.txt:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Python Service Server

```python
#!/usr/bin/env python3
import rospy
from my_package.srv import AddTwoInts, AddTwoIntsResponse

def handle_add(req):
    result = req.a + req.b
    rospy.loginfo(f"{req.a} + {req.b} = {result}")
    return AddTwoIntsResponse(result)

def server():
    rospy.init_node('add_server')
    service = rospy.Service('add_two_ints', AddTwoInts, handle_add)
    rospy.loginfo("Service ready")
    rospy.spin()

if __name__ == '__main__':
    server()
```

## Python Service Client

```python
#!/usr/bin/env python3
import rospy
from my_package.srv import AddTwoInts
import sys

def client(a, b):
    rospy.wait_for_service('add_two_ints')
    try:
        add = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        response = add(a, b)
        return response.sum
    except rospy.ServiceException as e:
        rospy.logerr(f"Service failed: {e}")

if __name__ == '__main__':
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
        rospy.init_node('add_client')
        result = client(a, b)
        rospy.loginfo(f"Result: {result}")
```

## C++ Service Server

```cpp
#include <ros/ros.h>
#include <my_package/AddTwoInts.h>

bool add(my_package::AddTwoInts::Request &req,
         my_package::AddTwoInts::Response &res)
{
    res.sum = req.a + req.b;
    ROS_INFO("Request: %ld + %ld = %ld", req.a, req.b, res.sum);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_server");
    ros::NodeHandle nh;
    
    ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
    ROS_INFO("Service ready");
    ros::spin();
    
    return 0;
}
```

## C++ Service Client

```cpp
#include <ros/ros.h>
#include <my_package/AddTwoInts.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_client");
    
    if (argc != 3)
    {
        ROS_ERROR("Usage: add_client A B");
        return 1;
    }
    
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<my_package::AddTwoInts>("add_two_ints");
    
    my_package::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    
    if (client.call(srv))
    {
        ROS_INFO("Result: %ld", srv.response.sum);
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return 1;
    }
    
    return 0;
}
```

---

# ACTION TEMPLATES

## Step 1: Define Action File (*.action)

**File**: `my_package/action/Countdown.action`
```
# Goal
int32 count
---
# Result
string message
---
# Feedback
int32 current
```

## Step 2: Update package.xml

```xml
<build_depend>message_generation</build_depend>
<build_depend>actionlib_msgs</build_depend>
<exec_depend>message_runtime</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
```

## Step 3: Update CMakeLists.txt

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  actionlib_msgs
  message_generation
)

# Add action files
add_action_files(
  FILES
  Countdown.action
)

# Generate messages (MUST include actionlib_msgs!)
generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

# IMPORTANT: Update catkin_package
catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs actionlib_msgs message_runtime
)
```

## Step 4: Build

**CRITICAL**: Must build after creating .action file!
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Python Action Server

```python
#!/usr/bin/env python3
import rospy
import actionlib
from my_package.msg import CountdownAction, CountdownFeedback, CountdownResult

class CountdownServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer(
            'countdown',
            CountdownAction,
            self.execute,
            False
        )
        self.server.start()
    
    def execute(self, goal):
        rate = rospy.Rate(1)
        feedback = CountdownFeedback()
        result = CountdownResult()
        
        for i in range(goal.count, -1, -1):
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return
            
            feedback.current = i
            self.server.publish_feedback(feedback)
            rospy.loginfo(f"Counting: {i}")
            rate.sleep()
        
        result.message = "Done!"
        self.server.set_succeeded(result)

if __name__ == '__main__':
    rospy.init_node('countdown_server')
    server = CountdownServer()
    rospy.spin()
```

## Python Action Client

```python
#!/usr/bin/env python3
import rospy
import actionlib
from my_package.msg import CountdownAction, CountdownGoal

def feedback_cb(feedback):
    rospy.loginfo(f"Feedback: {feedback.current}")

def client():
    client = actionlib.SimpleActionClient('countdown', CountdownAction)
    client.wait_for_server()
    
    goal = CountdownGoal()
    goal.count = 10
    
    client.send_goal(goal, feedback_cb=feedback_cb)
    client.wait_for_result()
    
    result = client.get_result()
    rospy.loginfo(f"Result: {result.message}")

if __name__ == '__main__':
    rospy.init_node('countdown_client')
    client()
```

---

# LAUNCH FILE TEMPLATES

## Basic Launch File

```xml
<launch>
    <!-- Single node -->
    <node pkg="my_package" type="my_script.py" name="my_node" output="screen"/>
</launch>
```

## Multiple Nodes Launch

```xml
<launch>
    <!-- Multiple nodes -->
    <node pkg="my_package" type="publisher.py" name="pub" output="screen"/>
    <node pkg="my_package" type="subscriber.py" name="sub" output="screen"/>
    
    <!-- With parameters -->
    <node pkg="my_package" type="controller.py" name="ctrl" output="screen">
        <param name="speed" value="1.0"/>
        <param name="rate" value="10"/>
    </node>
</launch>
```

## Launch with Arguments

```xml
<launch>
    <arg name="robot_name" default="robot1"/>
    <arg name="speed" default="1.0"/>
    
    <node pkg="my_package" type="robot.py" name="$(arg robot_name)" output="screen">
        <param name="speed" value="$(arg speed)"/>
    </node>
</launch>
```

Usage:
```bash
roslaunch my_package robot.launch robot_name:=robot2 speed:=2.0
```

## Launch with Namespaces

```xml
<launch>
    <group ns="robot1">
        <node pkg="my_package" type="controller.py" name="controller" output="screen"/>
    </group>
    
    <group ns="robot2">
        <node pkg="my_package" type="controller.py" name="controller" output="screen"/>
    </group>
</launch>
```

## Launch with Remapping

```xml
<launch>
    <node pkg="my_package" type="controller.py" name="controller" output="screen">
        <remap from="/cmd_vel" to="/robot1/cmd_vel"/>
    </node>
</launch>
```

---

# TURTLESIM MOVEMENT PATTERNS

## Start Turtlesim
```bash
rosrun turtlesim turtlesim_node
```

## Move Straight

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_straight')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

twist = Twist()
twist.linear.x = 2.0
twist.angular.z = 0.0

while not rospy.is_shutdown():
    pub.publish(twist)
    rate.sleep()
```

## Move in Circle

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_circle')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

twist = Twist()
twist.linear.x = 2.0
twist.angular.z = 1.0  # Non-zero = circle

while not rospy.is_shutdown():
    pub.publish(twist)
    rate.sleep()
```

## Move in Square

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

rospy.init_node('move_square')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

twist = Twist()

for i in range(4):
    # Forward
    twist.linear.x = 2.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(2.0)
    
    # Stop
    twist.linear.x = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)
    
    # Turn 90 degrees
    twist.angular.z = math.pi / 2
    pub.publish(twist)
    rospy.sleep(1.0)
    
    # Stop
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)
```

## Move in Spiral

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_spiral')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

twist = Twist()
radius = 0.5
angular_speed = 2.0

while not rospy.is_shutdown() and radius < 10:
    twist.linear.x = radius * angular_speed
    twist.angular.z = angular_speed
    pub.publish(twist)
    
    radius += 0.01  # Increase radius
    rate.sleep()

# Stop
twist.linear.x = 0.0
twist.angular.z = 0.0
pub.publish(twist)
```

## Move in Figure-8

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_figure8')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

twist = Twist()

# First circle (counter-clockwise)
for i in range(100):
    twist.linear.x = 2.0
    twist.angular.z = 1.0
    pub.publish(twist)
    rate.sleep()

# Second circle (clockwise)
for i in range(100):
    twist.linear.x = 2.0
    twist.angular.z = -1.0
    pub.publish(twist)
    rate.sleep()
```

## Move in Diamond

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

rospy.init_node('move_diamond')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

twist = Twist()

# Rotate 45 degrees first
twist.angular.z = math.pi / 4
pub.publish(twist)
rospy.sleep(1)
twist.angular.z = 0.0
pub.publish(twist)

# Draw square (appears as diamond)
for i in range(4):
    twist.linear.x = 2.0
    pub.publish(twist)
    rospy.sleep(2)
    
    twist.linear.x = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)
    
    twist.angular.z = math.pi / 2
    pub.publish(twist)
    rospy.sleep(1)
    
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)
```

## Spawn New Turtle

```python
#!/usr/bin/env python3
import rospy
from turtlesim.srv import Spawn

rospy.init_node('spawn_turtle')
rospy.wait_for_service('/spawn')

spawn = rospy.ServiceProxy('/spawn', Spawn)
spawn(5.0, 5.0, 0.0, 'turtle2')  # x, y, theta, name

rospy.loginfo("Turtle2 spawned!")
```

## Get Turtle Position

```python
#!/usr/bin/env python3
import rospy
from turtlesim.msg import Pose

def pose_callback(msg):
    rospy.loginfo(f"Position: ({msg.x:.2f}, {msg.y:.2f})")
    rospy.loginfo(f"Angle: {msg.theta:.2f}")

rospy.init_node('get_position')
rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
rospy.spin()
```

## Go To Specific Point

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def go_to_goal(x, y):
    rospy.init_node('go_to_goal')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(10)
    
    twist = Twist()
    
    while not rospy.is_shutdown():
        # Calculate distance
        dx = x - current_pose.x
        dy = y - current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:
            break
        
        # Calculate angle
        desired_angle = math.atan2(dy, dx)
        angle_diff = desired_angle - current_pose.theta
        
        # Normalize angle
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Control
        twist.linear.x = 1.5 * distance
        twist.angular.z = 4.0 * angle_diff
        
        pub.publish(twist)
        rate.sleep()
    
    # Stop
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    go_to_goal(8.0, 8.0)
```

---

# LINE FOLLOWER CODE

## Simple Line Follower (Camera + OpenCV)

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower:
    def __init__(self):
        rospy.init_node('line_follower')
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.kp = 0.005  # Proportional gain
        self.linear_speed = 0.2
        
    def detect_line(self, image):
        height, width = image.shape[:2]
        
        # Region of interest (bottom half)
        roi = image[height//2:, :]
        
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Yellow line detection
        lower = np.array([20, 100, 100])
        upper = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                return cx, width // 2
        
        return None, width // 2
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        line_center, image_center = self.detect_line(cv_image)
        
        twist = Twist()
        
        if line_center is not None:
            error = image_center - line_center
            twist.linear.x = self.linear_speed
            twist.angular.z = self.kp * error
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    follower = LineFollower()
    rospy.spin()
```

## Line Follower with PID

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class PIDLineFollower:
    def __init__(self):
        rospy.init_node('pid_line_follower')
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # PID parameters
        self.kp = 0.008
        self.ki = 0.0001
        self.kd = 0.003
        
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = rospy.Time.now()
        
        self.base_speed = 0.3
    
    def pid_control(self, error):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        
        if dt == 0:
            return 0
        
        # PID calculation
        p = self.kp * error
        self.integral += error * dt
        i = self.ki * self.integral
        d = self.kd * (error - self.prev_error) / dt
        
        self.prev_error = error
        self.prev_time = current_time
        
        return p + i + d
    
    def detect_line(self, image):
        height, width = image.shape[:2]
        roi = image[height//2:, :]
        
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower = np.array([20, 100, 100])
        upper = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                return cx, width // 2
        
        return None, width // 2
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        line_center, image_center = self.detect_line(cv_image)
        
        twist = Twist()
        
        if line_center is not None:
            error = image_center - line_center
            angular_vel = self.pid_control(error)
            
            twist.linear.x = self.base_speed
            twist.angular.z = angular_vel
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Search
        
        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    follower = PIDLineFollower()
    rospy.spin()
```

---

# OBSTACLE AVOIDANCE CODE

## Wander Bot (Laser Scanner)

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

class WanderBot:
    def __init__(self):
        rospy.init_node('wander_bot')
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        self.obstacle_distance = 0.5
        self.min_distance = float('inf')
    
    def laser_callback(self, msg):
        # Get minimum distance in front
        front_ranges = msg.ranges[len(msg.ranges)//4 : 3*len(msg.ranges)//4]
        self.min_distance = min(front_ranges)
    
    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.min_distance > self.obstacle_distance:
                # Move forward
                twist.linear.x = 0.3
                twist.angular.z = 0.0
            else:
                # Turn
                twist.linear.x = 0.0
                twist.angular.z = random.choice([-1, 1]) * 0.5
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    bot = WanderBot()
    bot.run()
```

## Obstacle Avoidance (Ultrasonic)

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class ObstacleAvoider:
    def __init__(self):
        rospy.init_node('obstacle_avoider')
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.range_sub = rospy.Subscriber('/ultrasonic', Range, self.range_callback)
        
        self.threshold = 0.3
        self.distance = 1.0
    
    def range_callback(self, msg):
        self.distance = msg.range
    
    def run(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.distance > self.threshold:
                twist.linear.x = 0.2
            else:
                twist.angular.z = 0.8
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    avoider = ObstacleAvoider()
    avoider.run()
```

---

# COMPLETE EXAMPLES

## Example 1: Pub-Sub Communication

**Publisher (talker.py)**:
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(1)
    
    count = 0
    while not rospy.is_shutdown():
        msg = String()
        msg.data = f"Hello {count}"
        rospy.loginfo(msg.data)
        pub.publish(msg)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

**Subscriber (listener.py)**:
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"I heard: {msg.data}")

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
```

**Launch File**:
```xml
<launch>
    <node pkg="my_package" type="talker.py" name="talker" output="screen"/>
    <node pkg="my_package" type="listener.py" name="listener" output="screen"/>
</launch>
```

---

## Example 2: Service Call

**Server (add_server.py)**:
```python
#!/usr/bin/env python3
import rospy
from my_package.srv import AddTwoInts, AddTwoIntsResponse

def handle_add(req):
    return AddTwoIntsResponse(req.a + req.b)

rospy.init_node('add_server')
service = rospy.Service('add_two_ints', AddTwoInts, handle_add)
rospy.loginfo("Ready to add")
rospy.spin()
```

**Client (add_client.py)**:
```python
#!/usr/bin/env python3
import rospy
from my_package.srv import AddTwoInts

rospy.init_node('add_client')
rospy.wait_for_service('add_two_ints')

add = rospy.ServiceProxy('add_two_ints', AddTwoInts)
result = add(10, 20)
rospy.loginfo(f"10 + 20 = {result.sum}")
```

---

## Example 3: Keyboard Teleop

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleop:
    def __init__(self):
        rospy.init_node('keyboard_teleop')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
    
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        print("w: forward | s: back | a: left | d: right | x: quit")
        
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                twist = Twist()
                
                if key == 'w':
                    twist.linear.x = 0.5
                elif key == 's':
                    twist.linear.x = -0.5
                elif key == 'a':
                    twist.angular.z = 0.5
                elif key == 'd':
                    twist.angular.z = -0.5
                elif key == 'x':
                    break
                
                self.pub.publish(twist)
        finally:
            twist = Twist()
            self.pub.publish(twist)

if __name__ == '__main__':
    teleop = KeyboardTeleop()
    teleop.run()
```

---

## Example 4: Wall Follower

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.desired_distance = 0.5
    
    def callback(self, msg):
        # Get left side distance
        left_idx = len(msg.ranges) // 4
        left_distance = msg.ranges[left_idx]
        
        twist = Twist()
        error = left_distance - self.desired_distance
        
        twist.linear.x = 0.3
        twist.angular.z = -error * 2.0
        
        self.pub.publish(twist)

if __name__ == '__main__':
    follower = WallFollower()
    rospy.spin()
```

---

## Example 5: Multi-Robot Control

```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class MultiRobotController:
    def __init__(self):
        rospy.init_node('multi_robot_controller')
        
        self.pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        self.pub2 = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=10)
        
        rate = rospy.Rate(10)
        
        twist1 = Twist()
        twist2 = Twist()
        
        while not rospy.is_shutdown():
            # Robot 1: Circle
            twist1.linear.x = 1.0
            twist1.angular.z = 0.5
            
            # Robot 2: Straight
            twist2.linear.x = 1.0
            twist2.angular.z = 0.0
            
            self.pub1.publish(twist1)
            self.pub2.publish(twist2)
            rate.sleep()

if __name__ == '__main__':
    MultiRobotController()
```

---

# QUICK DEBUG COMMANDS

```bash
# Check if roscore is running
ps aux | grep roscore

# List all nodes
rosnode list

# List all topics
rostopic list

# See messages on a topic
rostopic echo /topic_name

# Get topic info
rostopic info /topic_name

# Publish to topic manually
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 1.0} angular: {z: 0.5}"

# Check message type
rostopic type /topic_name

# Check service list
rosservice list

# Call service
rosservice call /service_name arg1 arg2

# Kill node
rosnode kill /node_name

# Check node info
rosnode info /node_name

# See message structure
rosmsg show geometry_msgs/Twist

# See all message types
rosmsg list

# Check if custom service/action messages are generated
rosmsg show my_package/AddTwoInts      # For services
rosmsg show my_package/CountdownAction # For actions
rossrv show my_package/AddTwoInts      # Alternative for services
```

---

# 🚨 COMMON ERRORS & FIXES

## Error: "No module named 'my_package.srv'"

**Cause**: Service messages not generated

**Fix**:
```bash
# 1. Check package.xml has message_generation
# 2. Check CMakeLists.txt has add_service_files()
# 3. Rebuild:
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Error: "No module named 'my_package.msg'"

**Cause**: Action messages not generated

**Fix**:
```bash
# 1. Check package.xml has actionlib_msgs
# 2. Check CMakeLists.txt has add_action_files()
# 3. Rebuild:
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Error: "Unable to load manifest for package"

**Cause**: CMakeLists.txt or package.xml syntax error

**Fix**:
```bash
# Check for typos in XML tags
# Check for missing dependencies
# Try clean build:
cd ~/catkin_ws
rm -rf build/ devel/
catkin_make
```

## Verify Service/Action Messages Exist

```bash
# After catkin_make, check if messages were generated:
ls ~/catkin_ws/devel/lib/python3/dist-packages/my_package/srv/
ls ~/catkin_ws/devel/lib/python3/dist-packages/my_package/msg/

# Should see generated Python files like:
# _AddTwoInts.py
# _CountdownAction.py
```

---

# COMMON MESSAGE TYPES

```python
# Import messages
from std_msgs.msg import String, Int32, Float32, Bool
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from sensor_msgs.msg import LaserScan, Image, Range
from turtlesim.msg import Pose as TurtlePose

# Twist (movement)
twist = Twist()
twist.linear.x = 1.0    # forward/back
twist.linear.y = 0.0    # left/right
twist.linear.z = 0.0    # up/down
twist.angular.x = 0.0   # roll
twist.angular.y = 0.0   # pitch
twist.angular.z = 0.5   # yaw (rotation)

# Point
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 0.0

# String
msg = String()
msg.data = "Hello"

# Int32
msg = Int32()
msg.data = 42

# LaserScan
# Access in callback:
def callback(msg):
    ranges = msg.ranges  # List of distances
    min_dist = min(ranges)
```

---

# EXAM STRATEGY

## Quick Checklist

**Before coding:**
- [ ] Read problem carefully
- [ ] Identify: Pub? Sub? Service? Action?
- [ ] Identify: Which topics/messages?
- [ ] Sketch node structure

**While coding:**
- [ ] Use template as starting point
- [ ] Change topic names
- [ ] Change message types
- [ ] Test with `rostopic echo`

**Common fixes:**
```bash
# Not running? Source it!
source devel/setup.bash

# Not executable? Fix it!
chmod +x script.py

# Topic wrong? Check it!
rostopic list
```

---

## Code Pattern Recognition

| If question says... | Use this... |
|---------------------|-------------|
| "continuously publish" | Publisher |
| "listen to / receive" | Subscriber |
| "call on demand" | Service |
| "long task with updates" | Action |
| "move in circle/square" | Twist publisher to /cmd_vel |
| "detect obstacle" | LaserScan subscriber |
| "follow line" | Image subscriber + Twist publisher |
| "keyboard control" | Keyboard input + Twist publisher |
| "spawn turtle" | Service call to /spawn |
| "multiple nodes" | Launch file |

---

## Quick Copy-Paste Checklist

**Every Python script needs:**
```python
#!/usr/bin/env python3
import rospy

rospy.init_node('node_name')

# ... your code ...

if __name__ == '__main__':
    # your main code
```

**Publisher needs:**
```python
pub = rospy.Publisher('/topic', MsgType, queue_size=10)
rate = rospy.Rate(10)
# ... in loop:
pub.publish(msg)
rate.sleep()
```

**Subscriber needs:**
```python
def callback(msg):
    # process msg
rospy.Subscriber('/topic', MsgType, callback)
rospy.spin()
```

**Service server needs:**
```python
def handle(req):
    # process req
    return ResponseType(result)
rospy.Service('service_name', ServiceType, handle)
rospy.spin()
```

**Service client needs:**
```python
rospy.wait_for_service('service_name')
service = rospy.ServiceProxy('service_name', ServiceType)
response = service(args)
```

---

# YOU'RE READY! 🚀

**Remember:**
1. Copy template
2. Change names
3. Test with rostopic
4. Done!

**Most common exam patterns:**
- 70% Publisher/Subscriber
- 20% Movement control
- 10% Services

**Master pub/sub + Twist = Pass exam!**

Good luck! 💪
