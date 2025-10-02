# 🤖 ROS Crash Course: Zero to Midterm Ready

> **Your Mission**: Master ROS fundamentals in one intensive session
> 
> **Exam Date**: Tomorrow
> 
> **Current Knowledge**: Absolute Beginner
> 
> **Goal**: Understand core concepts + recognize exam patterns + code confidently

---

## 📚 Table of Contents

1. [Part 1: What is ROS?](#part-1-what-is-ros)
2. [Part 2: Core Architecture](#part-2-core-architecture)
3. [Part 3: Your First Node (Publisher/Subscriber)](#part-3-your-first-node-publishersubscriber)
4. [Part 4: Services](#part-4-services)
5. [Part 5: Actions](#part-5-actions)
6. [Part 6: Launch Files](#part-6-launch-files)
7. [Part 7: Exam Pattern Recognition](#part-7-exam-pattern-recognition)
8. [Part 8: Practice Problems](#part-8-practice-problems)
9. [Part 9: Quick Reference Cheat Sheet](#part-9-quick-reference-cheat-sheet)

---

## Part 1: What is ROS?

### The Simple Answer

**ROS (Robot Operating System)** is a framework for writing robot software. Think of it as:
- A **messaging system** that lets different programs talk to each other
- A **toolbox** with pre-built robot functionality
- A **standardized way** to organize robot code

### Why ROS Exists

Imagine building a robot from scratch:
- Camera needs to talk to object detection
- Object detection needs to talk to motion planning
- Motion planning needs to talk to motor controls

Without ROS: You'd write custom communication code for each connection.

With ROS: Everything talks through a standardized messaging system.

### Key Philosophy

> **"Don't build a monolithic robot program. Build small, independent programs (nodes) that communicate."**

---

## Part 2: Core Architecture

### The 5 Core Concepts You MUST Know

```
┌─────────────────────────────────────────────────────────┐
│                      ROS MASTER                         │
│           (roscore - the central coordinator)           │
└─────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
    ┌───────┐          ┌────────┐         ┌────────┐
    │ NODE  │──Topic──▶│  NODE  │──Topic──▶│  NODE  │
    │   A   │          │   B    │          │   C    │
    └───────┘          └────────┘         └────────┘
        │                   │
        └──Service Request──┘
```

---

### 1️⃣ **Nodes** 

**What**: Independent programs that do ONE thing well.

**Example**: 
- Node 1: Read camera data
- Node 2: Detect objects
- Node 3: Move robot

**Analogy**: Like apps on your phone - each does one job.

---

### 2️⃣ **Topics** (Publisher/Subscriber Pattern)

**What**: Named channels where nodes send/receive messages.

**Pattern**: 
- **Publisher** = Broadcasts messages (doesn't care who listens)
- **Subscriber** = Listens to messages (doesn't care who publishes)

**Analogy**: YouTube channel
- Publisher = Content creator uploading videos
- Subscriber = Viewers watching videos
- Topic = The channel name

**Example**:
```
Publisher Node              Topic: /temperature              Subscriber Node
   (Sensor)     ────────▶  [28.5°C, 29.1°C, ...]  ────────▶  (Display)
```

**Key Insight**: Many-to-many relationship
- One topic can have multiple publishers
- One topic can have multiple subscribers

---

### 3️⃣ **Services** (Request/Response Pattern)

**What**: Synchronous communication - ask a question, get an answer.

**Pattern**:
- **Client** = Asks a question
- **Server** = Provides an answer

**Analogy**: Restaurant order
- Client = Customer ordering food
- Server = Kitchen preparing and delivering food

**Example**:
```
Client Node          Service: /add_two_numbers         Server Node
   (Math)    ────Request: 5 + 3────▶                   (Calculator)
             ◀───Response: 8────────
```

**Key Difference from Topics**:
- Topics: Continuous stream (fire and forget)
- Services: One-time request-response (wait for answer)

---

### 4️⃣ **Actions** (Long-Running Tasks with Feedback)

**What**: For tasks that take time and need progress updates.

**Pattern**:
- **Client** = Requests task
- **Server** = Executes task + sends progress + sends result

**Analogy**: Pizza delivery
- Client = You ordering pizza
- Server = Restaurant
- Feedback = "Order received" → "Preparing" → "Out for delivery"
- Result = Pizza arrives (or cancellation)

**Example**:
```
Client Node              Action: /navigate_to_goal           Server Node
  (Brain)    ────Goal: Go to (5,5)────▶                      (Navigation)
             ◀────Feedback: 25% there───
             ◀────Feedback: 50% there───
             ◀────Feedback: 75% there───
             ◀────Result: Arrived!──────
```

**Key Features**:
- Can be **cancelled** mid-execution
- Provides **continuous feedback**
- Returns a **final result**

---

### 5️⃣ **Packages**

**What**: A folder containing related ROS code.

**Structure**:
```
my_robot_package/
├── CMakeLists.txt      # Build instructions
├── package.xml         # Package metadata
├── scripts/            # Python nodes (executable)
├── src/                # C++ source files
├── launch/             # Launch files
├── msg/                # Custom message definitions
├── srv/                # Custom service definitions
└── action/             # Custom action definitions
```

**Analogy**: Like a Python module or npm package

---

## Part 3: Your First Node (Publisher/Subscriber)

### 🎯 Exam Pattern Recognition

**If you see**: "Create a node that publishes/sends data continuously"
→ **Use**: Publisher

**If you see**: "Create a node that receives/listens to data"
→ **Use**: Subscriber

---

### Example Task: Temperature Monitoring System

**Scenario**: Create two nodes:
1. Publisher: Broadcasts temperature readings every second
2. Subscriber: Receives and displays temperature

---

### 📝 Python Implementation

#### **Publisher (temperature_sensor.py)**

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import random

def temperature_publisher():
    # 1. Initialize the node
    rospy.init_node('temperature_sensor', anonymous=True)
    
    # 2. Create a publisher
    #    - Topic name: '/temperature'
    #    - Message type: Float32
    #    - Queue size: 10 (buffer for messages)
    pub = rospy.Publisher('/temperature', Float32, queue_size=10)
    
    # 3. Set publishing rate (1 Hz = once per second)
    rate = rospy.Rate(1)
    
    # 4. Main loop
    rospy.loginfo("Temperature sensor started!")
    while not rospy.is_shutdown():
        # Generate random temperature
        temp = random.uniform(20.0, 35.0)
        
        # Log to console
        rospy.loginfo(f"Publishing temperature: {temp:.2f}°C")
        
        # Publish the message
        pub.publish(temp)
        
        # Sleep to maintain rate
        rate.sleep()

if __name__ == '__main__':
    try:
        temperature_publisher()
    except rospy.ROSInterruptException:
        pass
```

**📖 Code Breakdown**:

1. **`rospy.init_node('name')`**: Register this program as a ROS node
2. **`rospy.Publisher(topic, type, queue_size)`**: Create publisher object
3. **`rospy.Rate(hz)`**: Control loop frequency
4. **`pub.publish(data)`**: Send message
5. **`rate.sleep()`**: Wait to maintain frequency

---

#### **Subscriber (temperature_display.py)**

```python
#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callback(msg):
    """
    This function is called AUTOMATICALLY every time 
    a message arrives on the /temperature topic
    """
    rospy.loginfo(f"Received temperature: {msg.data:.2f}°C")
    
    # You can add logic here
    if msg.data > 30.0:
        rospy.logwarn("WARNING: Temperature is high!")

def temperature_subscriber():
    # 1. Initialize the node
    rospy.init_node('temperature_display', anonymous=True)
    
    # 2. Create a subscriber
    #    - Topic name: '/temperature'
    #    - Message type: Float32
    #    - Callback: function to call when message arrives
    rospy.Subscriber('/temperature', Float32, callback)
    
    # 3. Keep the program running
    rospy.loginfo("Temperature display started! Waiting for data...")
    rospy.spin()  # Infinite loop - waits for messages

if __name__ == '__main__':
    try:
        temperature_subscriber()
    except rospy.ROSInterruptException:
        pass
```

**📖 Code Breakdown**:

1. **`def callback(msg)`**: Function called when message arrives
2. **`msg.data`**: Access the actual data from the message
3. **`rospy.Subscriber(topic, type, callback)`**: Register subscriber
4. **`rospy.spin()`**: Keep node alive, processing callbacks

---

### 💻 C++ Implementation

#### **Publisher (temperature_sensor.cpp)**

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <cstdlib>
#include <ctime>

int main(int argc, char **argv)
{
    // 1. Initialize the node
    ros::init(argc, argv, "temperature_sensor");
    
    // 2. Create node handle (interface to ROS)
    ros::NodeHandle nh;
    
    // 3. Create publisher
    ros::Publisher pub = nh.advertise<std_msgs::Float32>("/temperature", 10);
    
    // 4. Set rate (1 Hz)
    ros::Rate rate(1);
    
    // Random seed
    srand(time(0));
    
    ROS_INFO("Temperature sensor started!");
    
    // 5. Main loop
    while (ros::ok())
    {
        // Create message
        std_msgs::Float32 msg;
        msg.data = 20.0 + (rand() % 15);
        
        // Log and publish
        ROS_INFO("Publishing temperature: %.2f°C", msg.data);
        pub.publish(msg);
        
        // Sleep
        rate.sleep();
    }
    
    return 0;
}
```

---

#### **Subscriber (temperature_display.cpp)**

```cpp
#include <ros/ros.h>
#include <std_msgs/Float32.h>

// Callback function
void callback(const std_msgs::Float32::ConstPtr& msg)
{
    ROS_INFO("Received temperature: %.2f°C", msg->data);
    
    if (msg->data > 30.0)
    {
        ROS_WARN("WARNING: Temperature is high!");
    }
}

int main(int argc, char **argv)
{
    // 1. Initialize node
    ros::init(argc, argv, "temperature_display");
    
    // 2. Create node handle
    ros::NodeHandle nh;
    
    // 3. Create subscriber
    ros::Subscriber sub = nh.subscribe("/temperature", 10, callback);
    
    ROS_INFO("Temperature display started! Waiting for data...");
    
    // 4. Keep running
    ros::spin();
    
    return 0;
}
```

---

### 🔧 How to Run This

```bash
# Terminal 1: Start ROS core
roscore

# Terminal 2: Run publisher
cd ~/catkin_ws
source devel/setup.bash
chmod +x src/my_package/scripts/temperature_sensor.py
rosrun my_package temperature_sensor.py

# Terminal 3: Run subscriber
cd ~/catkin_ws
source devel/setup.bash
chmod +x src/my_package/scripts/temperature_display.py
rosrun my_package temperature_display.py

# Terminal 4: Debug - see all topics
rostopic list

# See messages on a topic
rostopic echo /temperature

# See topic info
rostopic info /temperature
```

---

## Part 4: Services

### 🎯 Exam Pattern Recognition

**If you see**: "Calculate/compute something on demand" or "perform an action and return a result"
→ **Use**: Service

**Key words**: "call", "request", "on-demand", "compute and return"

---

### Example Task: Math Calculator Service

**Scenario**: Create a service that adds two numbers

---

### Step 1: Define Service (AddTwoInts.srv)

```
# Request (what client sends)
int64 a
int64 b
---
# Response (what server sends back)
int64 sum
```

**Location**: `my_package/srv/AddTwoInts.srv`

---

### 📝 Python Implementation

#### **Server (add_two_ints_server.py)**

```python
#!/usr/bin/env python3

import rospy
from my_package.srv import AddTwoInts, AddTwoIntsResponse

def handle_add_two_ints(req):
    """
    This function is called when a client makes a request
    """
    result = req.a + req.b
    rospy.loginfo(f"Request: {req.a} + {req.b} = {result}")
    
    # Return the response
    return AddTwoIntsResponse(result)

def add_two_ints_server():
    # 1. Initialize node
    rospy.init_node('add_two_ints_server')
    
    # 2. Create service
    #    - Service name: 'add_two_ints'
    #    - Service type: AddTwoInts
    #    - Handler function: handle_add_two_ints
    service = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    
    rospy.loginfo("Add Two Ints service ready!")
    rospy.spin()

if __name__ == '__main__':
    add_two_ints_server()
```

---

#### **Client (add_two_ints_client.py)**

```python
#!/usr/bin/env python3

import rospy
from my_package.srv import AddTwoInts
import sys

def add_two_ints_client(a, b):
    # 1. Wait for service to be available
    rospy.wait_for_service('add_two_ints')
    
    try:
        # 2. Create a proxy (handle) to the service
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        
        # 3. Call the service
        response = add_two_ints(a, b)
        
        # 4. Return the result
        return response.sum
        
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    if len(sys.argv) == 3:
        a = int(sys.argv[1])
        b = int(sys.argv[2])
    else:
        print("Usage: add_two_ints_client.py A B")
        sys.exit(1)
    
    rospy.init_node('add_two_ints_client')
    
    result = add_two_ints_client(a, b)
    rospy.loginfo(f"Result: {a} + {b} = {result}")
```

---

### 💻 C++ Implementation

#### **Server (add_two_ints_server.cpp)**

```cpp
#include <ros/ros.h>
#include <my_package/AddTwoInts.h>

// Service callback
bool handle_add_two_ints(my_package::AddTwoInts::Request &req,
                         my_package::AddTwoInts::Response &res)
{
    res.sum = req.a + req.b;
    ROS_INFO("Request: %ld + %ld = %ld", req.a, req.b, res.sum);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_server");
    ros::NodeHandle nh;
    
    // Create service
    ros::ServiceServer service = nh.advertiseService("add_two_ints", handle_add_two_ints);
    
    ROS_INFO("Add Two Ints service ready!");
    ros::spin();
    
    return 0;
}
```

---

#### **Client (add_two_ints_client.cpp)**

```cpp
#include <ros/ros.h>
#include <my_package/AddTwoInts.h>
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_client");
    
    if (argc != 3)
    {
        ROS_ERROR("Usage: add_two_ints_client A B");
        return 1;
    }
    
    ros::NodeHandle nh;
    
    // Create client
    ros::ServiceClient client = nh.serviceClient<my_package::AddTwoInts>("add_two_ints");
    
    // Prepare request
    my_package::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    
    // Call service
    if (client.call(srv))
    {
        ROS_INFO("Result: %ld + %ld = %ld", srv.request.a, srv.request.b, srv.response.sum);
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

## Part 5: Actions

### 🎯 Exam Pattern Recognition

**If you see**: "Long-running task with progress updates" or "task that can be cancelled"
→ **Use**: Action

**Key words**: "monitor progress", "feedback", "cancellable", "navigate", "move to goal"

---

### Example Task: Countdown Action

**Scenario**: Count down from N to 0, providing progress feedback

---

### Step 1: Define Action (Countdown.action)

```
# Goal (what client requests)
int32 count
---
# Result (final outcome)
string message
int32 final_count
---
# Feedback (progress updates)
int32 current_count
float32 percent_complete
```

**Location**: `my_package/action/Countdown.action`

---

### 📝 Python Implementation

#### **Server (countdown_server.py)**

```python
#!/usr/bin/env python3

import rospy
import actionlib
from my_package.msg import CountdownAction, CountdownFeedback, CountdownResult

class CountdownServer:
    def __init__(self):
        # Create action server
        self.server = actionlib.SimpleActionServer(
            'countdown',                    # Action name
            CountdownAction,                # Action type
            self.execute,                   # Callback function
            False                           # Don't auto-start
        )
        self.server.start()
        rospy.loginfo("Countdown action server started!")
    
    def execute(self, goal):
        """
        This function is called when a client sends a goal
        """
        rospy.loginfo(f"Received goal: Count from {goal.count} to 0")
        
        feedback = CountdownFeedback()
        result = CountdownResult()
        rate = rospy.Rate(1)  # 1 Hz
        
        # Countdown loop
        for i in range(goal.count, -1, -1):
            # Check if client requested cancellation
            if self.server.is_preempt_requested():
                rospy.loginfo("Goal cancelled by client")
                self.server.set_preempted()
                return
            
            # Publish feedback
            feedback.current_count = i
            feedback.percent_complete = (goal.count - i) / float(goal.count) * 100
            self.server.publish_feedback(feedback)
            
            rospy.loginfo(f"Counting: {i} ({feedback.percent_complete:.1f}% complete)")
            rate.sleep()
        
        # Send result
        result.message = "Countdown complete!"
        result.final_count = 0
        self.server.set_succeeded(result)
        rospy.loginfo("Goal succeeded!")

if __name__ == '__main__':
    rospy.init_node('countdown_server')
    server = CountdownServer()
    rospy.spin()
```

---

#### **Client (countdown_client.py)**

```python
#!/usr/bin/env python3

import rospy
import actionlib
from my_package.msg import CountdownAction, CountdownGoal

def feedback_callback(feedback):
    """
    Called automatically when server sends feedback
    """
    rospy.loginfo(f"Feedback: Count={feedback.current_count}, Progress={feedback.percent_complete:.1f}%")

def countdown_client(count):
    # Create action client
    client = actionlib.SimpleActionClient('countdown', CountdownAction)
    
    # Wait for server
    rospy.loginfo("Waiting for countdown server...")
    client.wait_for_server()
    rospy.loginfo("Connected to server!")
    
    # Create goal
    goal = CountdownGoal()
    goal.count = count
    
    # Send goal
    rospy.loginfo(f"Sending goal: Count from {count} to 0")
    client.send_goal(goal, feedback_cb=feedback_callback)
    
    # Wait for result (with timeout)
    client.wait_for_result(rospy.Duration(count + 5))
    
    # Get result
    result = client.get_result()
    state = client.get_state()
    
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo(f"Success! {result.message}")
    else:
        rospy.logwarn("Goal did not succeed")

if __name__ == '__main__':
    rospy.init_node('countdown_client')
    countdown_client(10)  # Count from 10 to 0
```

---

## Part 6: Launch Files

### 🎯 What Launch Files Do

**Problem**: Running multiple nodes requires opening many terminals.

**Solution**: Launch files start multiple nodes with one command.

---

### Example: Multi-Node Launch File

```xml
<!-- File: my_package/launch/temperature_system.launch -->
<launch>
    <!-- Launch the temperature sensor node -->
    <node 
        pkg="my_package" 
        type="temperature_sensor.py" 
        name="temp_sensor" 
        output="screen"
    />
    
    <!-- Launch the temperature display node -->
    <node 
        pkg="my_package" 
        type="temperature_display.py" 
        name="temp_display" 
        output="screen"
    />
    
    <!-- Launch with parameters -->
    <node 
        pkg="my_package" 
        type="temperature_sensor.py" 
        name="temp_sensor_2" 
        output="screen"
    >
        <param name="rate" value="2.0"/>
        <param name="min_temp" value="15.0"/>
    </node>
</launch>
```

**Usage**:
```bash
roslaunch my_package temperature_system.launch
```

---

### Launch File Syntax Breakdown

```xml
<launch>
    <!-- Basic node -->
    <node pkg="PACKAGE" type="EXECUTABLE" name="NODE_NAME" output="screen"/>
    
    <!-- With parameters -->
    <node pkg="PACKAGE" type="EXECUTABLE" name="NODE_NAME">
        <param name="param1" value="10"/>
        <remap from="/old_topic" to="/new_topic"/>
    </node>
    
    <!-- Include another launch file -->
    <include file="$(find other_package)/launch/other.launch"/>
    
    <!-- Set arguments -->
    <arg name="robot_name" default="robot1"/>
    <node pkg="pkg" type="node.py" name="$(arg robot_name)"/>
</launch>
```

---

## Part 7: Exam Pattern Recognition

### 🎯 Decision Tree for Exam Questions

```
START: Read the problem
    │
    ├─> "Continuous data streaming/publishing?"
    │   └─> YES → Use TOPIC (Publisher/Subscriber)
    │
    ├─> "One-time request with immediate response?"
    │   └─> YES → Use SERVICE
    │
    ├─> "Long task with progress updates or cancellable?"
    │   └─> YES → Use ACTION
    │
    ├─> "Multiple nodes need to start together?"
    │   └─> YES → Use LAUNCH FILE
    │
    └─> "Coordinate robot movement (shapes, paths)?"
        └─> YES → Use PUBLISHER to /cmd_vel topic
```

---

### 🐢 Turtlesim: The Exam's Best Friend

**Why turtlesim?**: Most exams use it because it's visual and simple.

#### Common Turtlesim Topics

```bash
# Turtle position (subscribe to track position)
/turtle1/pose → geometry_msgs/Pose2D
  - x, y: position
  - theta: orientation (radians)
  - linear_velocity, angular_velocity

# Turtle movement (publish to control)
/turtle1/cmd_vel → geometry_msgs/Twist
  - linear.x: forward/backward speed
  - angular.z: rotation speed
```

---

### 📐 Movement Patterns Cheat Sheet

#### **1. Straight Line**
```python
twist = Twist()
twist.linear.x = 2.0   # Move forward at 2 m/s
twist.angular.z = 0.0  # No rotation
pub.publish(twist)
```

#### **2. Circle**
```python
twist = Twist()
twist.linear.x = 2.0   # Forward speed
twist.angular.z = 1.0  # Constant rotation
pub.publish(twist)
```

#### **3. Spiral (Expanding)**
```python
radius = 0.5
while not rospy.is_shutdown():
    twist.linear.x = radius * angular_speed
    twist.angular.z = angular_speed
    pub.publish(twist)
    
    radius += 0.1  # Increase radius over time
    rate.sleep()
```

#### **4. Square**
```python
# Repeat 4 times:
# 1. Move forward
# 2. Stop
# 3. Rotate 90 degrees
# 4. Stop

for i in range(4):
    # Move forward
    twist.linear.x = 2.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(2.0)  # For 2 seconds
    
    # Stop
    twist.linear.x = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)
    
    # Rotate 90 degrees
    twist.linear.x = 0.0
    twist.angular.z = 1.57  # π/2 radians = 90 degrees
    pub.publish(twist)
    rospy.sleep(1.0)
    
    # Stop
    twist.angular.z = 0.0
    pub.publish(twist)
```

#### **5. Diamond (45° Square)**
```python
# Same as square, but rotate 45° first, then rotate 90° at each corner
# Initial rotation
twist.angular.z = 0.785  # π/4 radians = 45 degrees
pub.publish(twist)
rospy.sleep(1.0)

# Then follow square pattern
```

#### **6. Figure-8**
```python
# Two circles in opposite directions
# Circle 1: Counter-clockwise
for i in range(100):
    twist.linear.x = 2.0
    twist.angular.z = 1.0
    pub.publish(twist)
    rate.sleep()

# Circle 2: Clockwise
for i in range(100):
    twist.linear.x = 2.0
    twist.angular.z = -1.0  # Negative = opposite direction
    pub.publish(twist)
    rate.sleep()
```

---

## Part 8: Practice Problems

### 🎓 Problem 1: Spawn and Move Turtle

**Task**: Spawn a new turtle and make it move in a circle.

<details>
<summary>🔍 Click to see solution</summary>

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

def spawn_and_move():
    rospy.init_node('turtle_spawner')
    
    # Spawn new turtle
    rospy.wait_for_service('/spawn')
    spawn = rospy.ServiceProxy('/spawn', Spawn)
    spawn(5.0, 5.0, 0.0, 'turtle2')  # x, y, theta, name
    
    # Create publisher for new turtle
    pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    # Move in circle
    twist = Twist()
    twist.linear.x = 2.0
    twist.angular.z = 1.0
    
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    spawn_and_move()
```

**Key Steps**:
1. Use `/spawn` service to create turtle
2. Publish to `/turtle2/cmd_vel` (new turtle's topic)
3. Set linear.x and angular.z for circular motion

</details>

---

### 🎓 Problem 2: Spiral Motion

**Task**: Make turtle1 move in an expanding spiral.

<details>
<summary>🔍 Click to see solution</summary>

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def spiral_motion():
    rospy.init_node('spiral_turtle')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    
    twist = Twist()
    radius = 0.5
    angular_speed = 2.0
    
    while not rospy.is_shutdown() and radius < 10:
        # Spiral: increase radius over time
        twist.linear.x = radius * angular_speed
        twist.angular.z = angular_speed
        
        pub.publish(twist)
        radius += 0.01  # Gradual expansion
        rate.sleep()
    
    # Stop
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)

if __name__ == '__main__':
    spiral_motion()
```

**Key Insight**: 
- Spiral = circle with increasing radius
- `linear.x = radius * angular.z` maintains spiral

</details>

---

### 🎓 Problem 3: Diamond Shape

**Task**: Make turtle trace a diamond (45° rotated square).

<details>
<summary>🔍 Click to see solution</summary>

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import math

def diamond_shape():
    rospy.init_node('diamond_turtle')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    
    twist = Twist()
    
    # Initial 45-degree rotation
    twist.angular.z = math.pi / 4  # 45 degrees
    pub.publish(twist)
    rospy.sleep(1)
    twist.angular.z = 0.0
    pub.publish(twist)
    
    # Draw square (which looks like diamond due to rotation)
    for i in range(4):
        # Move forward
        twist.linear.x = 2.0
        pub.publish(twist)
        rospy.sleep(2)
        
        # Stop
        twist.linear.x = 0.0
        pub.publish(twist)
        rospy.sleep(0.5)
        
        # Turn 90 degrees
        twist.angular.z = math.pi / 2
        pub.publish(twist)
        rospy.sleep(1)
        
        # Stop rotation
        twist.angular.z = 0.0
        pub.publish(twist)
        rospy.sleep(0.5)

if __name__ == '__main__':
    diamond_shape()
```

**Key Insight**: Diamond = Square rotated 45°

</details>

---

### 🎓 Problem 4: Combined Motion (Spiral → Diamond → Up)

**Task**: Turtle does spiral, then diamond, then moves straight up.

<details>
<summary>🔍 Click to see solution</summary>

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

def combined_motion():
    rospy.init_node('combined_turtle')
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rate = rospy.Rate(10)
    
    twist = Twist()
    
    # Phase 1: Spiral
    rospy.loginfo("Phase 1: Spiral")
    radius = 0.5
    angular_speed = 2.0
    
    for i in range(100):
        twist.linear.x = radius * angular_speed
        twist.angular.z = angular_speed
        pub.publish(twist)
        radius += 0.02
        rate.sleep()
    
    # Stop
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(1)
    
    # Phase 2: Diamond (simplified - just 4 moves)
    rospy.loginfo("Phase 2: Diamond")
    
    # Rotate to 45 degrees
    twist.angular.z = math.pi / 4
    pub.publish(twist)
    rospy.sleep(1)
    twist.angular.z = 0.0
    pub.publish(twist)
    
    # Draw 4 sides
    for i in range(4):
        twist.linear.x = 1.5
        pub.publish(twist)
        rospy.sleep(1.5)
        
        twist.linear.x = 0.0
        twist.angular.z = math.pi / 2
        pub.publish(twist)
        rospy.sleep(1)
        
        twist.angular.z = 0.0
        pub.publish(twist)
    
    rospy.sleep(1)
    
    # Phase 3: Move straight up
    rospy.loginfo("Phase 3: Moving up")
    
    # Orient to 90 degrees (straight up)
    target_angle = math.pi / 2
    angle_diff = target_angle - current_pose.theta
    
    # Rotate to face up
    twist.angular.z = 1.0 if angle_diff > 0 else -1.0
    pub.publish(twist)
    rospy.sleep(abs(angle_diff))
    
    # Stop rotation
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(0.5)
    
    # Move forward (which is now "up")
    twist.linear.x = 2.0
    pub.publish(twist)
    rospy.sleep(2)
    
    # Final stop
    twist.linear.x = 0.0
    pub.publish(twist)
    
    rospy.loginfo("Motion complete!")

if __name__ == '__main__':
    combined_motion()
```

**Key Insights**:
1. Break complex motion into phases
2. Use pose subscriber to track orientation
3. Stop between phases for clean transitions

</details>

---

### 🎓 Problem 5: Service-Based Movement

**Task**: Create a service that moves turtle to a specific (x, y) coordinate.

<details>
<summary>🔍 Click to see solution</summary>

**Step 1**: Define service (`GoToPoint.srv`)
```
# Request
float64 x
float64 y
---
# Response
bool success
string message
```

**Step 2**: Server

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_package.srv import GoToPoint, GoToPointResponse
import math

current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def handle_go_to_point(req):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    twist = Twist()
    
    # Calculate distance and angle
    while True:
        dx = req.x - current_pose.x
        dy = req.y - current_pose.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1:  # Close enough
            break
        
        # Calculate desired angle
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
    
    return GoToPointResponse(True, f"Reached ({req.x}, {req.y})")

def go_to_point_server():
    rospy.init_node('go_to_point_server')
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    service = rospy.Service('go_to_point', GoToPoint, handle_go_to_point)
    rospy.loginfo("Go to point service ready!")
    rospy.spin()

if __name__ == '__main__':
    go_to_point_server()
```

**Step 3**: Client

```python
#!/usr/bin/env python3

import rospy
from my_package.srv import GoToPoint

def go_to_point_client(x, y):
    rospy.wait_for_service('go_to_point')
    try:
        go_to_point = rospy.ServiceProxy('go_to_point', GoToPoint)
        response = go_to_point(x, y)
        rospy.loginfo(response.message)
    except rospy.ServiceException as e:
        rospy.logerr(f"Service failed: {e}")

if __name__ == '__main__':
    rospy.init_node('go_to_point_client')
    go_to_point_client(8.0, 8.0)
```

**Key Insights**:
1. Use pose subscriber to get current position
2. Calculate angle using `atan2(dy, dx)`
3. Proportional control: speed ∝ distance, rotation ∝ angle difference

</details>

---

## Part 9: Quick Reference Cheat Sheet

### 📋 Command Line Tools

```bash
# ROS Core
roscore                          # Start ROS master

# Nodes
rosnode list                     # List all running nodes
rosnode info /node_name          # Info about specific node
rosnode kill /node_name          # Kill a node

# Topics
rostopic list                    # List all topics
rostopic echo /topic_name        # Print messages on topic
rostopic info /topic_name        # Info about topic
rostopic hz /topic_name          # Frequency of messages
rostopic pub /topic_name TYPE DATA  # Publish to topic manually

# Services
rosservice list                  # List all services
rosservice call /service_name    # Call a service
rosservice info /service_name    # Info about service

# Packages
rospack list                     # List all packages
rospack find package_name        # Find package location
rosrun package_name node_name    # Run a node
roslaunch package_name file.launch  # Launch file

# Build
cd ~/catkin_ws
catkin_make                      # Build workspace
source devel/setup.bash          # Source workspace

# Turtlesim
rosrun turtlesim turtlesim_node  # Start turtlesim
rosrun turtlesim turtle_teleop_key  # Keyboard control
```

---

### 📋 Python Template Structure

#### **Publisher Template**
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String  # Change based on message type

def publisher():
    rospy.init_node('node_name', anonymous=True)
    pub = rospy.Publisher('topic_name', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hello"
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
```

#### **Subscriber Template**
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(f"Received: {msg.data}")

def subscriber():
    rospy.init_node('node_name', anonymous=True)
    rospy.Subscriber('topic_name', String, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```

---

### 📋 C++ Template Structure

#### **Publisher Template**
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::String>("topic_name", 10);
    ros::Rate rate(10);
    
    while (ros::ok())
    {
        std_msgs::String msg;
        msg.data = "Hello";
        pub.publish(msg);
        rate.sleep();
    }
    
    return 0;
}
```

#### **Subscriber Template**
```cpp
#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Received: %s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_name");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("topic_name", 10, callback);
    ros::spin();
    return 0;
}
```

---

### 📋 Common Message Types

```python
# Strings
from std_msgs.msg import String
msg = String()
msg.data = "text"

# Numbers
from std_msgs.msg import Int32, Float32
msg = Int32()
msg.data = 42

# Twist (for movement)
from geometry_msgs.msg import Twist
twist = Twist()
twist.linear.x = 1.0   # forward/back
twist.linear.y = 0.0   # left/right (usually 0)
twist.linear.z = 0.0   # up/down (usually 0)
twist.angular.x = 0.0  # roll (usually 0)
twist.angular.y = 0.0  # pitch (usually 0)
twist.angular.z = 1.0  # yaw (rotation)

# Pose (position + orientation)
from turtlesim.msg import Pose
# pose.x, pose.y, pose.theta
```

---

### 📋 Exam Strategy Checklist

**Before you start coding:**
- [ ] Read the problem twice
- [ ] Identify: Publisher/Subscriber, Service, or Action?
- [ ] Identify: Which topics/services to use?
- [ ] Sketch the flow diagram
- [ ] List the nodes you need to create

**While coding:**
- [ ] Start with template code
- [ ] Test frequently (`rostopic echo`, `rosnode list`)
- [ ] Use `rospy.loginfo()` for debugging
- [ ] Remember to `chmod +x` Python scripts
- [ ] Source workspace before running

**Common mistakes to avoid:**
- [ ] Forgetting to call `rospy.init_node()`
- [ ] Not making Python files executable
- [ ] Wrong topic names (check with `rostopic list`)
- [ ] Not sourcing workspace (`source devel/setup.bash`)
- [ ] Forgetting `rospy.spin()` in subscribers
- [ ] Not handling angles correctly (radians vs degrees)

---

## 🎯 Final Tips for Tomorrow

### Time Management
1. **Skim all questions first** (2 min)
2. **Do easy questions first** (publisher/subscriber)
3. **Leave complex patterns for last** (combined motions)

### Debugging Quick Wins
```bash
# If node not found:
source devel/setup.bash
chmod +x scripts/*.py

# If topic not working:
rostopic list          # Check if topic exists
rostopic echo /topic   # See if messages are publishing

# If roscore not running:
roscore &              # Run in background
```

### Mental Model Check

**Can you answer these?**
1. What's the difference between a topic and a service?
   - Topic: Continuous streaming, many-to-many
   - Service: One-time request-response, one-to-one

2. When do I use `rospy.spin()` vs `rate.sleep()`?
   - `spin()`: In subscribers (wait for callbacks)
   - `rate.sleep()`: In publishers (control loop rate)

3. How do I make turtle move in a circle?
   - Set linear.x (forward) and angular.z (rotation) both non-zero

4. How do I know current turtle position?
   - Subscribe to `/turtle1/pose`

---

## 🚀 You're Ready!

Remember:
- **ROS is just programs talking to each other**
- **Topics = continuous data flow**
- **Services = ask and answer**
- **Actions = long tasks with feedback**
- **Launch files = start multiple nodes**

You've got this! Trust the patterns, use the templates, and code confidently.

---

**Good luck on your midterm! 🎓**

---

## 📚 Additional Practice Resources

### Practice Questions to Test Yourself

1. **Explain to yourself**: What happens when you run `roslaunch`? (Hint: starts roscore + multiple nodes)

2. **Code without looking**: Write a publisher that sends "Hello" every 2 seconds

3. **Debug this**: Why isn't my subscriber receiving messages?
   - Check: Is roscore running?
   - Check: Is publisher running?
   - Check: Are topic names matching?
   - Check: `rostopic echo /topic_name`

4. **Design challenge**: You need a robot that:
   - Reads sensor data continuously → **Subscriber**
   - Processes data and sends commands → **Publisher**
   - Calculates path on demand → **Service**
   - Navigates to goal with progress → **Action**

### Your Next Steps After the Exam

1. Explore `tf` (transformations between coordinate frames)
2. Learn about `rosbridge` (connect ROS to web apps)
3. Study `rosbag` (record and replay ROS data)
4. Practice with real robots (if available)
5. Build a complete project (e.g., line follower, object tracker)

---

**Remember**: This crash course covers the essentials. ROS is vast, but you now know enough to tackle most midterm questions. Focus on understanding the patterns, practice the templates, and you'll do great!

**Pro tip**: Right before the exam, re-read Part 7 (Exam Pattern Recognition) and Part 9 (Quick Reference). These two sections are your exam superpowers!

Good luck! 🍀
