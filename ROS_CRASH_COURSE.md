# 🤖 ROS Crash Course: Zero to Midterm Ready

> **Your Mission**: Master ROS fundamentals in one intensive session
> 
> **Exam Date**: Tomorrow
> 
> **Current Knowledge**: Absolute Beginner
> 
> **Goal**: Complete syllabus coverage + coding mastery + exam confidence

---

## 📚 Table of Contents

### 📖 **MODULE 1: Introduction to Robotics (4 Hours)**
1. [M1.1: What is a Robot?](#m11-what-is-a-robot)
2. [M1.2: Seven Criteria of Defining a Robot](#m12-seven-criteria-of-defining-a-robot)
3. [M1.3: Robot Controllers & Components](#m13-robot-controllers--components)
4. [M1.4: Robot Instructions & Vocabularies](#m14-robot-instructions--vocabularies)
5. [M1.5: RSVP - Robot Scenario Visual Planning](#m15-rsvp---robot-scenario-visual-planning)
6. [M1.6: Pseudocode & Flowcharting](#m16-pseudocode--flowcharting)
7. [M1.7: State Charts for Robots](#m17-state-charts-for-robots)

### � **MODULE 2: Introduction to ROS (5 Hours)**
1. [M2.1: ROS Basics & ROS Equation](#m21-ros-basics--ros-equation)
2. [M2.2: History of ROS](#m22-history-of-ros)
3. [M2.3: Sensors & Robots Supporting ROS](#m23-sensors--robots-supporting-ros)
4. [M2.4: ROS Architecture & Concepts](#m24-ros-architecture--concepts)
5. [M2.5: roslaunch, catkin, Workspaces](#m25-roslaunch-catkin-workspaces)
6. [M2.6: ROS Packages](#m26-ros-packages)
7. [M2.7: Names, Namespaces, and Remapping](#m27-names-namespaces-and-remapping)
8. [M2.8: Coordinate Transforms (TF)](#m28-coordinate-transforms-tf)
9. [M2.9: Poses, Positions, and Orientations](#m29-poses-positions-and-orientations)

### 📖 **MODULE 3: ROS Programming (5 Hours)**
1. [M3.1: Topics - Publishing](#m31-topics---publishing)
2. [M3.2: Topics - Subscribing](#m32-topics---subscribing)
3. [M3.3: Custom Message Types](#m33-custom-message-types)
4. [M3.4: Services - Definition](#m34-services---definition)
5. [M3.5: Services - Implementation](#m35-services---implementation)
6. [M3.6: Actions - Complete Guide](#m36-actions---complete-guide)

### 📖 **MODULE 4: Robots Sensing and Moving (4 Hours)**
1. [M4.1: Robots and Simulators](#m41-robots-and-simulators)
2. [M4.2: Gazebo Simulator](#m42-gazebo-simulator)
3. [M4.3: Wander-Bot (Sensing & Actuation)](#m43-wander-bot-sensing--actuation)
4. [M4.4: Teleop-Bot (Keyboard Control)](#m44-teleop-bot-keyboard-control)
5. [M4.5: Motion Generator & Velocity Ramps](#m45-motion-generator--velocity-ramps)

### 📖 **MODULE 5: Perception and Behavior (4 Hours)**
1. [M5.1: Follow-Bot Overview](#m51-follow-bot-overview)
2. [M5.2: Acquiring Images](#m52-acquiring-images)
3. [M5.3: Detecting the Line](#m53-detecting-the-line)
4. [M5.4: Following the Line](#m54-following-the-line)
5. [M5.5: Complete Line Follower Implementation](#m55-complete-line-follower-implementation)

### 🎯 **Exam Preparation**
1. [Exam Pattern Recognition](#exam-pattern-recognition)
2. [Practice Problems](#practice-problems)
3. [Quick Reference Cheat Sheet](#quick-reference-cheat-sheet)

---

---

# 📘 MODULE 1: Introduction to Robotics

## M1.1: What is a Robot?

### The Academic Definition

A **robot** is a programmable, autonomous or semi-autonomous machine that can:
- **Sense** its environment
- **Process** information
- **Act** based on that information

### The Practical Definition

A robot = **Sensors + Brain + Actuators**

```
┌──────────────────────────────────────────────┐
│                   ROBOT                      │
│                                              │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐ │
│  │ SENSORS │───▶│  BRAIN  │───▶│ACTUATORS│ │
│  │(Input)  │    │(Process)│    │(Output) │ │
│  └─────────┘    └─────────┘    └─────────┘ │
│       │              │              │       │
│   Camera,        Computer,      Motors,     │
│   Lidar,         Logic,          Servos,    │
│   IMU            Decisions       Grippers   │
└──────────────────────────────────────────────┘
```

---

## M1.2: Seven Criteria of Defining a Robot

### 1️⃣ **Sensing (Perception)**
- **What**: Ability to gather information about the environment
- **Examples**: Cameras, ultrasonic sensors, touch sensors, GPS
- **Exam Focus**: "Robot must detect obstacles" → sensing requirement

### 2️⃣ **Mobility (Movement)**
- **What**: Ability to move in the environment
- **Types**: 
  - Wheeled (cars, rovers)
  - Legged (humanoids, quadrupeds)
  - Flying (drones)
  - Swimming (underwater robots)
- **Exam Focus**: "Make robot move in patterns" → mobility

### 3️⃣ **Energy (Power)**
- **What**: Power source to operate
- **Examples**: Batteries, solar panels, wired power
- **Key Concept**: Autonomy depends on battery life

### 4️⃣ **Intelligence (Brain)**
- **What**: Ability to make decisions
- **Levels**:
  - **Reactive**: Direct sensor-to-action (if obstacle → stop)
  - **Deliberative**: Plan then execute (calculate path → follow)
  - **Hybrid**: Combination of both
- **Exam Focus**: State machines, behavior trees

### 5️⃣ **Programmability**
- **What**: Can be instructed to perform different tasks
- **Key Point**: Not hardwired - can be reprogrammed
- **Exam Example**: "Write a program to make robot follow line"

### 6️⃣ **Manipulation**
- **What**: Ability to interact with objects
- **Examples**: Robotic arms, grippers, suction cups
- **Not Required**: Not all robots manipulate (vacuum robots don't)

### 7️⃣ **Communication**
- **What**: Ability to exchange information
- **Types**:
  - Human-Robot (display, speech)
  - Robot-Robot (swarm coordination)
  - Robot-Server (cloud connectivity)
- **ROS Context**: Nodes communicating via topics/services

---

## M1.3: Robot Controllers & Components

### Major Components of a Robot Controller

```
┌─────────────────────────────────────────────────┐
│         ROBOT CONTROLLER ARCHITECTURE           │
├─────────────────────────────────────────────────┤
│                                                 │
│  ┌──────────────┐         ┌──────────────┐    │
│  │ INPUT LAYER  │         │ OUTPUT LAYER │    │
│  │              │         │              │    │
│  │ • Sensors    │         │ • Motors     │    │
│  │ • Encoders   │         │ • Servos     │    │
│  │ • Cameras    │         │ • LEDs       │    │
│  └──────┬───────┘         └──────▲───────┘    │
│         │                        │             │
│         ▼                        │             │
│  ┌──────────────────────────────────────┐     │
│  │     PROCESSING LAYER (CPU/MCU)       │     │
│  │                                      │     │
│  │  • Read Sensors                      │     │
│  │  • Run Algorithms                    │     │
│  │  • Make Decisions                    │     │
│  │  • Generate Control Signals          │     │
│  └──────────────────────────────────────┘     │
│         ▲                        │             │
│         │                        ▼             │
│  ┌──────────────┐         ┌──────────────┐    │
│  │  POWER       │         │  MEMORY      │    │
│  │  MANAGEMENT  │         │  STORAGE     │    │
│  └──────────────┘         └──────────────┘    │
└─────────────────────────────────────────────────┘
```

### 1. **Microcontroller/Computer (Brain)**
- **Examples**: Arduino, Raspberry Pi, Jetson Nano
- **Functions**: Execute code, process sensor data, control actuators

### 2. **Motor Drivers**
- **What**: Interface between brain and motors
- **Why Needed**: MCU outputs low power (5V, 20mA) → Motors need high power (12V, 2A)
- **Examples**: L298N, TB6612FNG

### 3. **Sensors (Input Devices)**
- **Proximity**: Ultrasonic, IR, Lidar
- **Vision**: Camera, depth camera
- **Motion**: IMU (gyroscope + accelerometer), encoder
- **Environment**: Temperature, humidity, gas

### 4. **Actuators (Output Devices)**
- **Motors**: DC, Servo, Stepper
- **Others**: Grippers, speakers, LEDs

### 5. **Power Supply**
- **Battery**: LiPo, Li-ion, NiMH
- **Voltage Regulation**: Buck/Boost converters

### 6. **Communication Modules**
- **Wireless**: WiFi, Bluetooth, Zigbee
- **Wired**: USB, UART, I2C, SPI

---

## M1.4: Robot Instructions & Vocabularies

### Robot Instruction Hierarchy

```
HIGH LEVEL:  "Go to the kitchen"
             ↓ (Task Planning)
MID LEVEL:   "Move forward 5m, turn right 90°, move 3m"
             ↓ (Motion Planning)
LOW LEVEL:   "Set motor1=150rpm, motor2=150rpm"
             ↓ (Motor Control)
HARDWARE:    PWM signals to motor drivers
```

### Common Robot Vocabularies

#### **Motion Commands**
- **Linear Motion**: forward, backward, stop
- **Rotational Motion**: turn left, turn right, rotate
- **Speed**: velocity, acceleration, deceleration

#### **Sensor Queries**
- **Read**: get_sensor_value()
- **Check**: is_obstacle_detected()
- **Wait**: wait_until_sensor_triggered()

#### **Control Flow**
- **Sequential**: do A, then B, then C
- **Conditional**: if (condition) then action
- **Iterative**: repeat, loop, while

#### **ROS Vocabulary**
- **roscore**: Start ROS master
- **rosrun**: Run a single node
- **roslaunch**: Launch multiple nodes
- **rostopic**: Interact with topics
- **rosservice**: Call services
- **rosparam**: Get/set parameters

---

## M1.5: RSVP - Robot Scenario Visual Planning

### What is RSVP?

**RSVP = Robot Scenario Visual Planning**

A methodology to design robot behavior BEFORE coding:
1. **Map the Scenario**: Draw the environment
2. **Define Objects**: Identify all elements
3. **Plan Behavior**: Sketch robot actions
4. **Create State Chart**: Define states and transitions

---

### Step 1: Mapping the Scenario

**Example Scenario**: Robot vacuum cleaner

```
┌─────────────────────────────────────────┐
│         ROOM LAYOUT                     │
│                                         │
│  ┌────┐                                │
│  │Sofa│         ┌────┐                 │
│  └────┘         │Table│                │
│                 └────┘                 │
│     🤖 (Robot Start)                   │
│                                         │
│  ┌──┐      ░░░░░  (Dirty Area)        │
│  │Ch│      ░░░░░                       │
│  │ar│                                  │
│  └──┘            🚪 (Door)             │
└─────────────────────────────────────────┘

Legend:
🤖 = Robot
█ = Obstacle
░ = Dirt
🚪 = Boundary
```

---

### Step 2: Define Objects and Properties

| Object | Properties | Behaviors |
|--------|-----------|-----------|
| Robot | Position, Battery, Dirt Capacity | Move, Clean, Charge |
| Obstacles | Position, Size | Static |
| Dirt | Position, Amount | Removable |
| Charging Station | Position | Charges Robot |

---

### Step 3: Plan Robot Behavior

**High-Level Mission**: Clean entire room

**Sub-Tasks**:
1. Scan area for dirt
2. Navigate to dirty area
3. Clean dirt
4. Avoid obstacles
5. Return to charging station when low battery

---

## M1.6: Pseudocode & Flowcharting

### Pseudocode Example: Line Follower Robot

```
PROGRAM LineFollower

INITIALIZE:
    robot_speed = 100
    sensor_threshold = 500
    
START:
    WHILE robot is ON:
        left_sensor_value = READ(left_sensor)
        right_sensor_value = READ(right_sensor)
        
        IF both sensors detect line:
            move_forward()
            
        ELSE IF only left sensor detects line:
            turn_left()
            
        ELSE IF only right sensor detects line:
            turn_right()
            
        ELSE:
            // Lost the line
            stop()
            search_for_line()
        
        DELAY(10ms)
    END WHILE
    
END PROGRAM

FUNCTION move_forward():
    SET left_motor = robot_speed
    SET right_motor = robot_speed

FUNCTION turn_left():
    SET left_motor = robot_speed * 0.5
    SET right_motor = robot_speed

FUNCTION turn_right():
    SET left_motor = robot_speed
    SET right_motor = robot_speed * 0.5

FUNCTION stop():
    SET left_motor = 0
    SET right_motor = 0
```

---

### Flowchart Example: Obstacle Avoidance

```
         ┌─────────┐
         │  START  │
         └────┬────┘
              │
              ▼
      ┌───────────────┐
      │ Read Distance │
      │    Sensor     │
      └───────┬───────┘
              │
              ▼
         ╱         ╲
        ╱ Distance  ╲     NO
       ╱   < 20cm?  ╲─────────┐
       ╲             ╱         │
        ╲           ╱          │
         ╲         ╱           │
          └───┬───┘            │
          YES │                │
              ▼                │
       ┌──────────┐            │
       │   STOP   │            │
       └─────┬────┘            │
             │                 │
             ▼                 │
       ┌──────────┐            │
       │ Turn 90° │            │
       └─────┬────┘            │
             │                 │
             └────┐            │
                  │            │
                  ▼            ▼
             ┌─────────────────┐
             │  Move Forward   │
             └────────┬────────┘
                      │
                      └─────┐
                            │
                            ▼
                     ┌────────────┐
                     │   Repeat   │
                     └────────────┘
```

---

### Flowchart Symbols Reference

| Symbol | Name | Meaning |
|--------|------|---------|
| ⬭ (Oval) | Terminal | Start/End |
| ▭ (Rectangle) | Process | Action/Instruction |
| ⬠ (Diamond) | Decision | Yes/No Question |
| ⬟ (Parallelogram) | Input/Output | Read/Display |
| → (Arrow) | Flow Line | Direction |
| ⬡ (Hexagon) | Preparation | Loop initialization |

---

## M1.7: State Charts for Robots

### What are State Charts?

**State Charts** = Visual representation of robot behavior as a set of states and transitions

```
      [EVENT]
STATE ────────▶ NEW STATE
```

---

### Example 1: Simple Robot States

```
                    [Battery Low]
  ┌─────────────────────────────────┐
  │                                 │
  │                                 ▼
┌──┴───────┐  [Start]  ┌──────────────┐
│  IDLE    │─────────▶ │   WORKING    │
└──────────┘           └──┬───────────┘
  ▲                       │
  │     [Task Done]       │
  └───────────────────────┘
```

---

### Example 2: Line Follower State Chart

```
                              ┌─────────────┐
                      ┌──────▶│   FORWARD   │◀──────┐
                      │       └──────┬──────┘       │
                      │              │              │
         [Both on     │              │[Right off    │
          line]       │              │ line]        │
                      │              ▼              │
              ┌───────┴──────┐    ┌─────────────┐  │
       ┌─────▶│  TURN_LEFT   │    │ TURN_RIGHT  │──┘
       │      └──────────────┘    └─────────────┘
       │              ▲                  
       │              │                  
       │    [Left off │                  
       │     line]    │                  
       │              │                  
       │      ┌───────┴──────┐           
       └──────│   SEARCHING  │           
              └──────────────┘           
                [Both off line]
```

---

### Example 3: Warehouse Robot State Chart

```
                    START
                      │
                      ▼
                ┌──────────┐
         ┌─────▶│   IDLE   │◀────────┐
         │      └─────┬────┘         │
         │            │              │
         │   [Receive Order]         │
         │            │              │
         │            ▼              │
         │      ┌──────────┐         │
         │      │NAVIGATING│         │
         │      └─────┬────┘         │
         │            │              │
         │   [Reached Target]        │
         │            │              │
         │            ▼              │
         │      ┌──────────┐         │
         │      │ LOADING  │         │
         │      └─────┬────┘         │
         │            │              │
         │      [Load Complete]      │
         │            │              │
         │            ▼              │
         │      ┌──────────┐         │
         │      │RETURNING │         │
         │      └─────┬────┘         │
         │            │              │
         │   [Reached Base]          │
         │            │              │
         │            ▼              │
         │      ┌──────────┐         │
         │      │UNLOADING │         │
         │      └─────┬────┘         │
         │            │              │
         │   [Unload Complete]       │
         │            │              │
         └────────────┘              │
                                     │
              [Emergency Stop]       │
                      │              │
                      ▼              │
                ┌──────────┐         │
                │ STOPPED  │─────────┘
                └──────────┘
                 [Resume]
```

---

### State Chart Components

1. **State (Circle/Rectangle)**: Robot's current mode
2. **Transition (Arrow)**: Change from one state to another
3. **Event (Label on Arrow)**: What triggers the transition
4. **Action (Inside State)**: What robot does in that state

---

### Writing State Charts for Exams

**Question Pattern**: "Design a robot that..."

**Steps**:
1. **Identify all possible states** (idle, moving, turning, stopped)
2. **Identify events/conditions** (sensor triggers, timers, commands)
3. **Draw state circles**
4. **Connect with labeled arrows**
5. **Mark initial state** (usually IDLE or START)

---

## M1.8: Checking Robot Capabilities

### Capability Checklist

Before programming, verify your robot can:

#### **Sensing Capabilities**
- [ ] Can it detect obstacles? (Ultrasonic, Lidar, IR)
- [ ] Can it see? (Camera, depth sensor)
- [ ] Can it know its position? (Encoders, GPS, IMU)
- [ ] Can it detect lines? (IR line sensors)

#### **Actuation Capabilities**
- [ ] Can it move forward/backward?
- [ ] Can it turn left/right?
- [ ] Can it control speed precisely?
- [ ] Can it manipulate objects? (Gripper, arm)

#### **Processing Capabilities**
- [ ] Processing power sufficient? (CPU, RAM)
- [ ] Can it run ROS? (Memory, OS compatibility)
- [ ] Real-time capable? (Response time < threshold)

#### **Power Capabilities**
- [ ] Battery life sufficient for task?
- [ ] Can it return to charge autonomously?

#### **Communication Capabilities**
- [ ] Can it connect wirelessly? (WiFi, Bluetooth)
- [ ] Can it receive commands?
- [ ] Can it send status updates?

---

### Example Capability Analysis

**Task**: Create line-following robot

**Required Capabilities**:
✅ Line sensors (minimum 2, better with 3-5)
✅ Differential drive (2 motors)
✅ Motor speed control (PWM)
✅ Processing (basic MCU sufficient)
❌ Camera (not required - simpler sensors work)
❌ Gripper (not needed for this task)

---

---

# 📘 MODULE 2: Introduction to ROS

## M2.1: ROS Basics & ROS Equation

### **The ROS Equation**

```
ROS = Plumbing + Tools + Capabilities + Ecosystem
```

Let's break this down:

#### 1️⃣ **Plumbing** (Communication Infrastructure)
- **Publish/Subscribe** messaging (topics)
- **Request/Reply** messaging (services)
- **Long-running tasks** messaging (actions)

#### 2️⃣ **Tools** (Developer Utilities)
- **Visualization**: rviz (3D visualization), rqt_graph (node graph)
- **Debugging**: rostopic echo, rosnode info
- **Recording**: rosbag (record/replay data)
- **Simulation**: Gazebo integration

#### 3️⃣ **Capabilities** (Pre-built Functionality)
- **Navigation**: Path planning, obstacle avoidance
- **Manipulation**: Inverse kinematics, trajectory planning
- **Perception**: Object detection, SLAM (mapping)

#### 4️⃣ **Ecosystem** (Community)
- 1000+ packages available
- Active community support
- Standardized interfaces

---

### Why ROS? The Real-World Problem

**Without ROS**:
```
Camera ──[Custom Protocol 1]──▶ Object Detection
                                       │
                                [Custom Protocol 2]
                                       │
                                       ▼
Motion Planning ──[Custom Protocol 3]──▶ Motor Control
```
*Result*: Every connection is custom-built. Pain to debug and extend.

**With ROS**:
```
Camera ──[/camera/image]──▶ Object Detection ──[/detected_objects]──▶ Motion Planning
                                                                              │
                                                                       [/cmd_vel]
                                                                              │
                                                                              ▼
                                                                       Motor Control
```
*Result*: Standardized topics. Plug-and-play components.

---

## M2.2: History of ROS

### Timeline

| Year | Event | Significance |
|------|-------|--------------|
| **2007** | Started at Stanford (STanford Artificial Intelligence Robot project) | Initial development |
| **2008** | Willow Garage takes over | Professional development begins |
| **2010** | ROS 1.0 released | First stable release |
| **2012** | ROS Groovy | Industrial robotics support |
| **2014** | ROS Indigo | LTS (Long Term Support) |
| **2016** | ROS Kinetic | Ubuntu 16.04 support |
| **2018** | ROS Melodic | Ubuntu 18.04, Python 3 support |
| **2020** | **ROS 2 Foxy** | Complete rewrite, real-time, embedded |
| **2023** | ROS Noetic (final ROS 1) | Last ROS 1 LTS version |

### ROS 1 vs ROS 2

| Feature | ROS 1 | ROS 2 |
|---------|-------|-------|
| Architecture | Single master (roscore) | Distributed (DDS) |
| Real-time | Limited | Full support |
| Security | Minimal | Built-in |
| Multi-robot | Complex | Native support |
| Python | 2.7 & 3 | 3+ only |
| Exam Focus | **ROS 1 (most universities)** | Future |

**For Your Exam**: Focus on **ROS 1** (Melodic/Noetic)

---

## M2.3: Sensors & Robots Supporting ROS

### Popular Sensors with ROS Drivers

#### **Vision Sensors**
- **Camera**: USB webcam, RealSense, ZED
  - ROS Package: `usb_cam`, `realsense2_camera`
- **Lidar**: Hokuyo, RPLIDAR, Velodyne
  - ROS Package: `hokuyo_node`, `rplidar_ros`

#### **Motion Sensors**
- **IMU**: MPU6050, BNO055
  - ROS Package: `imu_tools`
- **GPS**: u-blox, Garmin
  - ROS Package: `nmea_navsat_driver`

#### **Proximity Sensors**
- **Ultrasonic**: HC-SR04
- **IR**: Sharp GP2Y0A21YK

---

### Popular Robots Supporting ROS

#### **Mobile Robots**
- **TurtleBot** (educational)
- **Husky** (outdoor)
- **Jackal** (research)
- **PR2** (research platform)

#### **Manipulators**
- **UR5/UR10** (Universal Robots)
- **Panda** (Franka Emika)
- **Baxter** (Rethink Robotics)

#### **Aerial Robots**
- **DJI** (with ROS wrappers)
- **PX4-based** drones

#### **Simulated Robots**
- **Turtlesim** (2D)
- **Stage** (2D)
- **Gazebo** (3D - most popular)

---

## M2.4: ROS Architecture & Concepts

### ROS Architecture Layers

```
┌─────────────────────────────────────────────────┐
│         APPLICATION LAYER                       │
│    (Your robot-specific code)                   │
├─────────────────────────────────────────────────┤
│         ROS MIDDLEWARE                          │
│    (Topics, Services, Actions, Parameters)      │
├─────────────────────────────────────────────────┤
│         OS LAYER                                │
│    (Linux, Ubuntu)                              │
└─────────────────────────────────────────────────┘
```

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

## M2.5: roslaunch, catkin, Workspaces

### Understanding the Workspace

```
~/catkin_ws/                    ← Workspace Root
├── build/                      ← Build files (auto-generated)
├── devel/                      ← Development space
│   ├── setup.bash             ← Source this to use workspace
│   └── lib/                   ← Compiled binaries
└── src/                        ← Source code (YOUR CODE HERE)
    ├── CMakeLists.txt         ← Top-level build config
    ├── package1/
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   ├── scripts/           ← Python nodes
    │   ├── src/               ← C++ source
    │   ├── launch/            ← Launch files
    │   ├── msg/               ← Custom messages
    │   └── srv/               ← Custom services
    └── package2/
```

---

### Catkin Build System

**What is Catkin?**
- ROS's build system (based on CMake)
- Compiles C++ code, generates message headers
- Manages dependencies between packages

**Build Commands**:
```bash
# Method 1: catkin_make (traditional)
cd ~/catkin_ws
catkin_make

# Method 2: catkin build (newer, better)
cd ~/catkin_ws
catkin build

# Clean build
catkin_make clean
```

**After Building**:
```bash
# MUST source the workspace to use it
source devel/setup.bash

# Add to ~/.bashrc for automatic sourcing
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

### roslaunch Deep Dive

**What is roslaunch?**
- Starts **multiple nodes** with one command
- Automatically starts `roscore` if not running
- Can set parameters, remap topics, include other launch files

**Basic Syntax**:
```xml
<launch>
    <!-- Launch a node -->
    <node pkg="package_name" type="executable_name" name="node_name" output="screen"/>
</launch>
```

**Advanced Launch File**:
```xml
<launch>
    <!-- Arguments (command-line inputs) -->
    <arg name="robot_name" default="robot1"/>
    <arg name="use_sim" default="true"/>
    
    <!-- Parameters (configuration) -->
    <param name="robot_description" textfile="$(find my_package)/urdf/robot.urdf"/>
    
    <!-- Node with parameters -->
    <node pkg="my_package" type="controller.py" name="$(arg robot_name)_controller">
        <param name="speed" value="1.0"/>
        <remap from="/cmd_vel" to="/$(arg robot_name)/cmd_vel"/>
    </node>
    
    <!-- Include another launch file -->
    <include file="$(find other_package)/launch/sensors.launch"/>
    
    <!-- Conditional launch -->
    <group if="$(arg use_sim)">
        <node pkg="gazebo_ros" type="gzserver" name="gazebo"/>
    </group>
</launch>
```

**Usage**:
```bash
# Launch with default arguments
roslaunch my_package robot.launch

# Launch with custom arguments
roslaunch my_package robot.launch robot_name:=robot2 use_sim:=false
```

---

## M2.6: ROS Packages

### What is a Package?

**Package** = Container for related ROS code

**Minimum Requirements**:
1. `package.xml` - Package metadata
2. `CMakeLists.txt` - Build instructions

---

### Creating a Package

```bash
# Navigate to workspace source
cd ~/catkin_ws/src

# Create package with dependencies
catkin_create_pkg my_robot_pkg rospy roscpp std_msgs geometry_msgs

# Structure created:
# my_robot_pkg/
# ├── CMakeLists.txt
# ├── package.xml
# ├── include/my_robot_pkg/
# └── src/
```

---

### package.xml Explained

```xml
<?xml version="1.0"?>
<package format="2">
  <name>my_robot_pkg</name>
  <version>1.0.0</version>
  <description>My robot package</description>
  
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>
  
  <!-- Build dependencies (needed during compilation) -->
  <buildtool_depend>catkin</buildtool_depend>
  
  <!-- Dependencies -->
  <depend>rospy</depend>
  <depend>roscpp</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  
  <!-- If you create custom messages -->
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
</package>
```

---

### CMakeLists.txt Key Sections

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(my_robot_pkg)

## Find catkin and dependencies
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  geometry_msgs
)

## Generate custom messages (if any)
# add_message_files(
#   FILES
#   MyMessage.msg
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

## Catkin package configuration
catkin_package(
  CATKIN_DEPENDS rospy roscpp std_msgs geometry_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Build C++ executables
add_executable(my_cpp_node src/my_cpp_node.cpp)
target_link_libraries(my_cpp_node ${catkin_LIBRARIES})

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/my_python_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

---

### Package Management Commands

```bash
# List all packages
rospack list

# Find package location
rospack find package_name

# Show package dependencies
rospack depends package_name

# Go to package directory
roscd package_name

# List files in package
rosls package_name
```

---

## M2.7: Names, Namespaces, and Remapping

### ROS Naming System

ROS uses a hierarchical naming system (like file paths).

#### **Types of Names**

1. **Base Name**: `cmd_vel`
2. **Relative Name**: `robot1/cmd_vel`
3. **Global Name**: `/robot1/cmd_vel` (starts with `/`)
4. **Private Name**: `~parameter` (node-specific)

---

### Name Resolution Example

```python
# Node name: /robot1/controller

rospy.Publisher('cmd_vel', Twist)          # → /robot1/cmd_vel (relative)
rospy.Publisher('/cmd_vel', Twist)         # → /cmd_vel (global)
rospy.Publisher('~cmd_vel', Twist)         # → /robot1/controller/cmd_vel (private)
```

---

### Namespaces

**Why?** Multiple robots, avoid name collisions

**Example: Two Robots**
```
/robot1/cmd_vel      ← Robot 1's velocity commands
/robot1/odom         ← Robot 1's odometry
/robot1/scan         ← Robot 1's laser scan

/robot2/cmd_vel      ← Robot 2's velocity commands
/robot2/odom         ← Robot 2's odometry
/robot2/scan         ← Robot 2's laser scan
```

**In Launch File**:
```xml
<launch>
    <!-- Robot 1 in namespace "robot1" -->
    <group ns="robot1">
        <node pkg="my_pkg" type="controller.py" name="controller"/>
    </group>
    
    <!-- Robot 2 in namespace "robot2" -->
    <group ns="robot2">
        <node pkg="my_pkg" type="controller.py" name="controller"/>
    </group>
</launch>
```

---

### Remapping

**Purpose**: Change topic/service names without modifying code

**Command Line**:
```bash
# Original: publishes to /cmd_vel
rosrun my_pkg controller.py

# Remapped: publishes to /robot1/cmd_vel
rosrun my_pkg controller.py cmd_vel:=/robot1/cmd_vel
```

**Launch File**:
```xml
<node pkg="my_pkg" type="controller.py" name="controller">
    <remap from="cmd_vel" to="robot1/cmd_vel"/>
</node>
```

**In Code**:
```python
rospy.init_node('controller', anonymous=True)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # Can be remapped externally
```

---

## M2.8: Coordinate Transforms (TF)

### What is TF?

**TF (Transform)** = System for tracking coordinate frames over time

**Why Needed?**
- Robot has many parts, each with its own coordinate frame
- Need to convert between frames (e.g., "where is object relative to robot base?")

---

### TF Tree Example: Mobile Robot

```
           map
            │
            │ (robot's position in world)
            │
        base_link  ← Robot's center
            │
      ┌─────┴─────┬──────────┐
      │           │          │
  left_wheel  right_wheel  laser
                               │
                          laser_scan
```

**Frame Relationships**:
- `map` → `base_link`: Robot's position in world
- `base_link` → `left_wheel`: Wheel relative to robot center
- `base_link` → `laser`: Laser sensor relative to robot center

---

### Using TF in Python

```python
#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PointStamped

def transform_point():
    rospy.init_node('tf_example')
    
    # Create TF listener
    listener = tf.TransformListener()
    
    # Wait for transform to be available
    listener.waitForTransform("/map", "/base_link", rospy.Time(), rospy.Duration(4.0))
    
    # Point in base_link frame
    point_base = PointStamped()
    point_base.header.frame_id = "base_link"
    point_base.header.stamp = rospy.Time(0)  # Latest available
    point_base.point.x = 1.0
    point_base.point.y = 0.5
    point_base.point.z = 0.0
    
    # Transform to map frame
    point_map = listener.transformPoint("/map", point_base)
    
    rospy.loginfo(f"Point in map frame: ({point_map.point.x}, {point_map.point.y})")

if __name__ == '__main__':
    transform_point()
```

---

### Broadcasting TF

```python
#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import TransformStamped

def broadcast_tf():
    rospy.init_node('tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Broadcast transform from base_link to laser
        br.sendTransform(
            (0.1, 0.0, 0.2),  # Translation (x, y, z)
            (0.0, 0.0, 0.0, 1.0),  # Rotation (quaternion)
            rospy.Time.now(),
            "laser",  # Child frame
            "base_link"  # Parent frame
        )
        rate.sleep()

if __name__ == '__main__':
    broadcast_tf()
```

---

### TF Commands

```bash
# View TF tree
rosrun tf view_frames

# Echo transform between two frames
rosrun tf tf_echo /map /base_link

# Monitor TF
rosrun rqt_tf_tree rqt_tf_tree
```

---

## M2.9: Poses, Positions, and Orientations

### Position vs Pose

- **Position**: Where something is (x, y, z)
- **Orientation**: Which way it's facing (rotation)
- **Pose**: Position + Orientation

---

### Representing Position

```python
from geometry_msgs.msg import Point

point = Point()
point.x = 1.0
point.y = 2.0
point.z = 0.0
```

---

### Representing Orientation

#### **2D (Turtlesim)**
```python
# Angle in radians
theta = 1.57  # 90 degrees

# 0° = facing right
# 90° (π/2) = facing up
# 180° (π) = facing left
# 270° (3π/2) = facing down
```

#### **3D (Quaternions)**
```python
from geometry_msgs.msg import Quaternion
import tf.transformations as tft

# Convert from Euler angles (roll, pitch, yaw) to quaternion
roll = 0.0
pitch = 0.0
yaw = 1.57  # 90 degrees

quaternion = tft.quaternion_from_euler(roll, pitch, yaw)

q = Quaternion()
q.x = quaternion[0]
q.y = quaternion[1]
q.z = quaternion[2]
q.w = quaternion[3]
```

**Why Quaternions?**
- Avoid gimbal lock
- Smooth interpolation
- Compact representation

**Don't worry about the math** - use `tf.transformations` library!

---

### Complete Pose Message

```python
from geometry_msgs.msg import Pose, Point, Quaternion
import tf.transformations as tft

pose = Pose()

# Position
pose.position = Point(1.0, 2.0, 0.0)

# Orientation (facing 90 degrees)
quaternion = tft.quaternion_from_euler(0, 0, 1.57)
pose.orientation = Quaternion(*quaternion)
```

---

### Turtlesim Pose

```python
from turtlesim.msg import Pose

def pose_callback(msg):
    rospy.loginfo(f"Position: ({msg.x}, {msg.y})")
    rospy.loginfo(f"Orientation: {msg.theta} rad")
    rospy.loginfo(f"Linear velocity: {msg.linear_velocity}")
    rospy.loginfo(f"Angular velocity: {msg.angular_velocity}")

rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
```

---

### Coordinate Systems

#### **Common Frames**

- **map**: Fixed world frame
- **odom**: Odometry frame (drifts over time)
- **base_link**: Robot's center
- **base_footprint**: Robot's ground projection
- **camera_link**: Camera frame
- **laser**: Laser scanner frame

#### **Conventions**

**ROS Standard (REP 103)**:
- **X**: Forward
- **Y**: Left
- **Z**: Up

```
         Z (up)
         │
         │
         │
         └───── X (forward)
        ╱
       ╱
      Y (left)
```

---

---

# 📘 MODULE 3: ROS Programming

## M3.1: Topics - Publishing

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

---

# 📘 MODULE 4: Robots Sensing and Moving

## M4.1: Robots and Simulators

### Why Use Simulators?

**Advantages**:
- ✅ Safe (no hardware damage)
- ✅ Fast prototyping
- ✅ Easy to test edge cases
- ✅ Free (no need for expensive robots)
- ✅ Can simulate multiple robots

**Disadvantages**:
- ❌ Not 100% accurate to real world
- ❌ Doesn't capture all sensor noise
- ❌ Sim-to-real gap (works in sim ≠ works in reality)

---

### Popular ROS Simulators

| Simulator | Type | Best For | Complexity |
|-----------|------|----------|------------|
| **Turtlesim** | 2D | Learning basics | ⭐ Easy |
| **Stage** | 2D | Multi-robot, navigation | ⭐⭐ Medium |
| **Gazebo** | 3D | Realistic physics, sensors | ⭐⭐⭐ Advanced |
| **Webots** | 3D | Commercial, accurate | ⭐⭐⭐ Advanced |

---

## M4.2: Gazebo Simulator

### What is Gazebo?

**Gazebo** = 3D robot simulator with realistic physics

**Features**:
- Physics simulation (gravity, friction, collisions)
- Sensor simulation (camera, lidar, IMU)
- URDF/SDF robot models
- Plugin system for custom behaviors

---

### Starting Gazebo with ROS

```bash
# Start empty Gazebo world
roslaunch gazebo_ros empty_world.launch

# Start with a specific world
roslaunch gazebo_ros empty_world.launch world_name:=/path/to/world.world

# Launch your robot in Gazebo
roslaunch my_robot_pkg spawn_robot.launch
```

---

### URDF (Unified Robot Description Format)

**URDF** = XML format to describe robot structure

**Simple Robot Example**:
```xml
<?xml version="1.0"?>
<robot name="simple_bot">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
  </link>
  
  <!-- Joint connecting wheel to base -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0.15 0.2 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
</robot>
```

**Key URDF Components**:
- **Link**: Physical part (body, wheel, sensor)
- **Joint**: Connection between links
- **Geometry**: Shape (box, cylinder, sphere, mesh)
- **Material**: Color and appearance
- **Inertial**: Mass and inertia (for physics)

---

### Gazebo Plugins for Robot Control

**Differential Drive Plugin** (most common for mobile robots):

```xml
<gazebo>
  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
    <alwaysOn>true</alwaysOn>
    <updateRate>50</updateRate>
    <leftJoint>left_wheel_joint</leftJoint>
    <rightJoint>right_wheel_joint</rightJoint>
    <wheelSeparation>0.4</wheelSeparation>
    <wheelDiameter>0.2</wheelDiameter>
    <torque>10</torque>
    <commandTopic>cmd_vel</commandTopic>
    <odometryTopic>odom</odometryTopic>
    <odometryFrame>odom</odometryFrame>
    <robotBaseFrame>base_link</robotBaseFrame>
  </plugin>
</gazebo>
```

**Now you can control the robot**:
```bash
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 1.0} angular: {z: 0.5}"
```

---

## M4.3: Wander-Bot (Sensing & Actuation)

### Concept: Wander-Bot

**Behavior**: Robot moves forward until obstacle detected, then turns and continues

**Algorithm**:
```
WHILE true:
    distance = read_sensor()
    
    IF distance > threshold:
        move_forward()
    ELSE:
        stop()
        turn_random_direction()
        move_forward()
```

---

### Wander-Bot Implementation (Python)

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import random

class WanderBot:
    def __init__(self):
        rospy.init_node('wander_bot')
        
        # Publisher for movement commands
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber for laser scan data
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        
        # Parameters
        self.obstacle_distance = 0.5  # meters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        
        # State
        self.min_distance = float('inf')
        
        rospy.loginfo("Wander-bot started!")
        
    def laser_callback(self, msg):
        """
        Called when laser scan data arrives
        msg.ranges = list of distances [front, left, back, right, ...]
        """
        # Get minimum distance in front of robot
        # Assuming laser covers 360 degrees
        front_ranges = msg.ranges[len(msg.ranges)//4 : 3*len(msg.ranges)//4]
        self.min_distance = min(front_ranges)
        
    def wander(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.min_distance > self.obstacle_distance:
                # No obstacle - move forward
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
                rospy.loginfo("Moving forward")
            else:
                # Obstacle detected - turn
                twist.linear.x = 0.0
                twist.angular.z = random.choice([-1, 1]) * self.angular_speed
                rospy.loginfo(f"Obstacle at {self.min_distance:.2f}m - Turning!")
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    try:
        bot = WanderBot()
        bot.wander()
    except rospy.ROSInterruptException:
        pass
```

---

### Wander-Bot with Ultrasonic Sensor

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class SimpleWanderBot:
    def __init__(self):
        rospy.init_node('simple_wander_bot')
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.range_sub = rospy.Subscriber('/ultrasonic', Range, self.range_callback)
        
        self.obstacle_distance = 0.3  # 30 cm
        self.current_distance = 1.0
        
    def range_callback(self, msg):
        self.current_distance = msg.range
        
    def wander(self):
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            twist = Twist()
            
            if self.current_distance > self.obstacle_distance:
                # Move forward
                twist.linear.x = 0.2
            else:
                # Turn in place
                twist.angular.z = 0.8
                
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    bot = SimpleWanderBot()
    bot.wander()
```

---

## M4.4: Teleop-Bot (Keyboard Control)

### Concept: Teleoperation

**Teleoperation** = Remote control of robot by human operator

**Components**:
1. **Keyboard Driver**: Reads keyboard input
2. **Motion Generator**: Converts keys to velocity commands

---

### Keyboard Driver Node

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
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.linear_speed = 0.5
        self.angular_speed = 1.0
        
        # Store original terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.key_bindings = {
            'w': (1, 0),   # Forward
            's': (-1, 0),  # Backward
            'a': (0, 1),   # Turn left
            'd': (0, -1),  # Turn right
            'q': (1, 1),   # Forward + Left
            'e': (1, -1),  # Forward + Right
            'z': (-1, 1),  # Backward + Left
            'c': (-1, -1), # Backward + Right
            ' ': (0, 0),   # Stop
        }
        
    def get_key(self):
        """Read a single keypress"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def run(self):
        print("Keyboard Teleop Controls:")
        print("---------------------------")
        print("w: forward    s: backward")
        print("a: turn left  d: turn right")
        print("q: fwd+left   e: fwd+right")
        print("z: back+left  c: back+right")
        print("SPACE: stop   x: quit")
        print("---------------------------")
        
        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                
                if key == 'x':
                    break
                    
                if key in self.key_bindings:
                    linear, angular = self.key_bindings[key]
                    
                    twist = Twist()
                    twist.linear.x = linear * self.linear_speed
                    twist.angular.z = angular * self.angular_speed
                    
                    self.cmd_vel_pub.publish(twist)
                    
        except Exception as e:
            print(e)
        finally:
            # Stop robot
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            # Restore terminal
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        teleop = KeyboardTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        pass
```

---

### Using Existing Teleop Packages

```bash
# Install teleop_twist_keyboard
sudo apt-get install ros-noetic-teleop-twist-keyboard

# Run it
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

---

## M4.5: Motion Generator & Velocity Ramps

### Why Velocity Ramps?

**Problem**: Instant velocity changes cause:
- Motor strain
- Wheel slipping
- Unstable motion
- Potential damage

**Solution**: Gradual acceleration/deceleration (ramping)

---

### Simple Velocity Ramp

```python
#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class VelocityRamper:
    def __init__(self):
        rospy.init_node('velocity_ramper')
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_smooth', Twist, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_vel_callback)
        
        # Current velocity
        self.current_linear = 0.0
        self.current_angular = 0.0
        
        # Target velocity
        self.target_linear = 0.0
        self.target_angular = 0.0
        
        # Acceleration limits (m/s²)
        self.linear_accel = 0.5
        self.angular_accel = 1.0
        
        # Control rate
        self.rate = rospy.Rate(50)  # 50 Hz
        self.dt = 1.0 / 50.0
        
    def cmd_vel_callback(self, msg):
        """Receive target velocity"""
        self.target_linear = msg.linear.x
        self.target_angular = msg.angular.z
        
    def ramp_velocity(self, current, target, max_accel, dt):
        """
        Gradually change current velocity towards target
        """
        error = target - current
        max_change = max_accel * dt
        
        if abs(error) < max_change:
            return target
        elif error > 0:
            return current + max_change
        else:
            return current - max_change
            
    def run(self):
        while not rospy.is_shutdown():
            # Ramp velocities
            self.current_linear = self.ramp_velocity(
                self.current_linear,
                self.target_linear,
                self.linear_accel,
                self.dt
            )
            
            self.current_angular = self.ramp_velocity(
                self.current_angular,
                self.target_angular,
                self.angular_accel,
                self.dt
            )
            
            # Publish smoothed velocity
            twist = Twist()
            twist.linear.x = self.current_linear
            twist.angular.z = self.current_angular
            self.cmd_vel_pub.publish(twist)
            
            self.rate.sleep()

if __name__ == '__main__':
    ramper = VelocityRamper()
    ramper.run()
```

---

### Advanced: PID-based Velocity Controller

```python
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.prev_error = 0.0
        self.integral = 0.0
        
    def update(self, setpoint, measured, dt):
        """
        PID control
        """
        error = setpoint - measured
        
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Update
        self.prev_error = error
        
        # Control output
        output = p_term + i_term + d_term
        return output
```

---

---

# 📘 MODULE 5: Perception and Behavior

## M5.1: Follow-Bot Overview

### Line Following Robot

**Task**: Robot follows a line on the ground

**Sensors**: Camera or IR line sensors

**Algorithm**:
1. **Acquire image** from camera
2. **Process image** to detect line
3. **Calculate error** (how far from center?)
4. **Generate control** (turn left/right to center line)
5. **Repeat**

---

### Types of Line Detection

| Method | Sensor | Complexity | Cost |
|--------|--------|------------|------|
| **IR Sensors** | 3-5 IR sensors | ⭐ Low | $ |
| **Camera (Simple)** | USB camera | ⭐⭐ Medium | $$ |
| **Camera (Advanced)** | Depth camera | ⭐⭐⭐ High | $$$ |

---

## M5.2: Acquiring Images

### Camera in ROS

**Image Message Types**:
- `sensor_msgs/Image`: Raw image data
- `sensor_msgs/CompressedImage`: JPEG compressed

---

### Publishing Camera Feed (USB Webcam)

```bash
# Install usb_cam package
sudo apt-get install ros-noetic-usb-cam

# Launch camera node
roslaunch usb_cam usb_cam-test.launch

# View images
rosrun rqt_image_view rqt_image_view
```

---

### Subscribing to Camera in Python

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber:
    def __init__(self):
        rospy.init_node('image_subscriber')
        
        # Bridge between ROS and OpenCV
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
    def image_callback(self, msg):
        """
        Called when new image arrives
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Display image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    subscriber = ImageSubscriber()
    rospy.spin()
```

---

## M5.3: Detecting the Line

### Line Detection with OpenCV

**Steps**:
1. Convert image to HSV color space
2. Threshold to isolate line color
3. Find contours
4. Calculate line center

---

### Line Detection Implementation

```python
#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetector:
    def __init__(self):
        rospy.init_node('line_detector')
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        
        # Line color range (HSV for yellow line)
        self.lower_yellow = np.array([20, 100, 100])
        self.upper_yellow = np.array([30, 255, 255])
        
    def detect_line(self, image):
        """
        Detect line in image and return center position
        Returns: (cx, cy) or None if no line found
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Threshold to get only yellow colors
        mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Get largest contour (the line)
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate center using moments
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                return (cx, cy), mask
        
        return None, mask
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect line
            result, mask = self.detect_line(cv_image)
            
            if result:
                cx, cy = result
                # Draw center point
                cv2.circle(cv_image, (cx, cy), 10, (0, 255, 0), -1)
                rospy.loginfo(f"Line center at: ({cx}, {cy})")
            else:
                rospy.logwarn("No line detected!")
            
            # Display
            cv2.imshow("Original", cv_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    detector = LineDetector()
    rospy.spin()
```

---

### Adjusting Color Thresholds

Use trackbars to tune HSV values:

```python
def tune_hsv():
    """
    Interactive HSV tuning tool
    """
    rospy.init_node('hsv_tuner')
    
    cap = cv2.VideoCapture(0)  # Webcam
    
    # Create window with trackbars
    cv2.namedWindow('HSV Tuner')
    cv2.createTrackbar('H_low', 'HSV Tuner', 0, 179, lambda x: None)
    cv2.createTrackbar('S_low', 'HSV Tuner', 0, 255, lambda x: None)
    cv2.createTrackbar('V_low', 'HSV Tuner', 0, 255, lambda x: None)
    cv2.createTrackbar('H_high', 'HSV Tuner', 179, 179, lambda x: None)
    cv2.createTrackbar('S_high', 'HSV Tuner', 255, 255, lambda x: None)
    cv2.createTrackbar('V_high', 'HSV Tuner', 255, 255, lambda x: None)
    
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break
            
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get trackbar values
        h_low = cv2.getTrackbarPos('H_low', 'HSV Tuner')
        s_low = cv2.getTrackbarPos('S_low', 'HSV Tuner')
        v_low = cv2.getTrackbarPos('V_low', 'HSV Tuner')
        h_high = cv2.getTrackbarPos('H_high', 'HSV Tuner')
        s_high = cv2.getTrackbarPos('S_high', 'HSV Tuner')
        v_high = cv2.getTrackbarPos('V_high', 'HSV Tuner')
        
        # Apply mask
        lower = np.array([h_low, s_low, v_low])
        upper = np.array([h_high, s_high, v_high])
        mask = cv2.inRange(hsv, lower, upper)
        
        # Display
        cv2.imshow('Original', frame)
        cv2.imshow('Mask', mask)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
```

---

## M5.4: Following the Line

### Control Strategy: Proportional Control

**Error** = Desired position - Actual position

**Control Output** = K_p × Error

```
Image width = 640 pixels
Desired center = 320 pixels (middle)
Actual center = 400 pixels (line is to the right)

Error = 320 - 400 = -80 pixels

Angular velocity = K_p × (-80) = negative (turn right)
```

---

### Line Follower Implementation

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
        
        # Subscribers and Publishers
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Control parameters
        self.linear_speed = 0.2  # m/s
        self.kp = 0.005  # Proportional gain
        
        # Image dimensions
        self.image_center = 320  # Will be updated with actual image width
        
        # Line color (yellow)
        self.lower_bound = np.array([20, 100, 100])
        self.upper_bound = np.array([30, 255, 255])
        
        rospy.loginfo("Line follower started!")
        
    def detect_line_center(self, image):
        """
        Detect line and return x-coordinate of center
        """
        # Get only bottom half of image (where line is)
        height, width = image.shape[:2]
        self.image_center = width // 2
        roi = image[height//2:, :]
        
        # Convert to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Threshold
        mask = cv2.inRange(hsv, self.lower_bound, self.upper_bound)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Get largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get center
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                return cx, mask
                
        return None, mask
        
    def image_callback(self, msg):
        """
        Process image and generate control commands
        """
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect line
            line_center, mask = self.detect_line_center(cv_image)
            
            # Create twist message
            twist = Twist()
            
            if line_center is not None:
                # Calculate error
                error = self.image_center - line_center
                
                # Proportional control
                twist.linear.x = self.linear_speed
                twist.angular.z = self.kp * error
                
                rospy.loginfo(f"Error: {error}, Angular: {twist.angular.z:.3f}")
                
                # Visualize
                cv2.circle(cv_image, (line_center, cv_image.shape[0]//2), 10, (0, 255, 0), -1)
                cv2.line(cv_image, (self.image_center, 0), (self.image_center, cv_image.shape[0]), (0, 0, 255), 2)
            else:
                # Line lost - stop
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                rospy.logwarn("Line lost!")
            
            # Publish control command
            self.cmd_vel_pub.publish(twist)
            
            # Display
            cv2.imshow("Line Follower", cv_image)
            cv2.imshow("Mask", mask)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    try:
        follower = LineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

---

## M5.5: Complete Line Follower Implementation

### Advanced Line Follower with PID Control

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
        
        # PID variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = rospy.Time.now()
        
        # Speed parameters
        self.base_speed = 0.3
        self.max_angular = 2.0
        
        # Image center
        self.image_center = 320
        
        # Line color (adjust for your line color)
        self.lower_bound = np.array([0, 0, 200])     # White line
        self.upper_bound = np.array([180, 30, 255])
        
    def pid_control(self, error):
        """
        PID controller
        """
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        
        if dt == 0:
            return 0
        
        # Proportional
        p_term = self.kp * error
        
        # Integral
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Update for next iteration
        self.prev_error = error
        self.prev_time = current_time
        
        # Total control
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(-self.max_angular, min(self.max_angular, output))
        
        return output
        
    def detect_line(self, image):
        """
        Detect line using edge detection and Hough lines
        """
        height, width = image.shape[:2]
        self.image_center = width // 2
        
        # Region of interest (bottom half)
        roi = image[height//2:, :]
        
        # Convert to grayscale
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Get largest contour
            largest = max(contours, key=cv2.contourArea)
            
            # Get bounding rect center
            x, y, w, h = cv2.boundingRect(largest)
            cx = x + w // 2
            
            return cx, edges
            
        return None, edges
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            line_center, processed = self.detect_line(cv_image)
            
            twist = Twist()
            
            if line_center is not None:
                # Calculate error
                error = self.image_center - line_center
                
                # PID control
                angular_vel = self.pid_control(error)
                
                # Reduce speed when turning sharply
                speed_reduction = 1.0 - (abs(angular_vel) / self.max_angular) * 0.5
                
                twist.linear.x = self.base_speed * speed_reduction
                twist.angular.z = angular_vel
                
                # Visualization
                cv2.circle(cv_image, (line_center, cv_image.shape[0]//2), 10, (0, 255, 0), -1)
                cv2.line(cv_image, (self.image_center, 0), (self.image_center, cv_image.shape[0]), (0, 0, 255), 2)
                
                status_text = f"Error: {error:. 0f} | Angular: {angular_vel:.2f}"
                cv2.putText(cv_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            else:
                # Line lost - stop or search
                twist.linear.x = 0.0
                twist.angular.z = 0.5  # Slowly rotate to search
                
                cv2.putText(cv_image, "LINE LOST - SEARCHING", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            self.cmd_vel_pub.publish(twist)
            
            cv2.imshow("Line Follower", cv_image)
            cv2.imshow("Processed", processed)
            cv2.waitKey(1)
            
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    try:
        follower = PIDLineFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

---

### Launch File for Line Follower

```xml
<!-- File: line_follower.launch -->
<launch>
    <!-- Start camera -->
    <node pkg="usb_cam" type="usb_cam_node" name="camera">
        <param name="video_device" value="/dev/video0"/>
        <param name="image_width" value="640"/>
        <param name="image_height" value="480"/>
        <param name="framerate" value="30"/>
    </node>
    
    <!-- Start line follower -->
    <node pkg="my_robot_pkg" type="line_follower.py" name="line_follower" output="screen">
        <param name="kp" value="0.008"/>
        <param name="ki" value="0.0001"/>
        <param name="kd" value="0.003"/>
        <param name="base_speed" value="0.3"/>
    </node>
    
    <!-- Optional: Record data -->
    <node pkg="rosbag" type="record" name="recorder" args="-a -o line_follower_data"/>
</launch>
```

---

### Testing with Gazebo

```xml
<!-- gazebo_line_world.launch -->
<launch>
    <!-- Start Gazebo with line track world -->
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find my_robot_pkg)/worlds/line_track.world"/>
    </include>
    
    <!-- Spawn robot -->
    <node name="spawn_robot" pkg="gazebo_ros" type="spawn_model"
          args="-urdf -model my_robot -param robot_description"/>
    
    <!-- Start line follower -->
    <node pkg="my_robot_pkg" type="line_follower.py" name="line_follower" output="screen"/>
</launch>
```

---

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

---

---

# 🎯 EXAM SURVIVAL GUIDE: Last-Minute Review

## Module-by-Module Summary

### MODULE 1: Introduction (Key Points)
✅ **Seven Criteria**: Sensing, Mobility, Energy, Intelligence, Programmability, Manipulation, Communication
✅ **Controller Components**: MCU/Computer, Motor Drivers, Sensors, Actuators, Power, Communication
✅ **RSVP**: Map scenario → Define objects → Plan behavior → Create state charts
✅ **State Charts**: Visual representation of robot states and transitions

### MODULE 2: ROS Basics (Key Points)
✅ **ROS Equation**: Plumbing + Tools + Capabilities + Ecosystem
✅ **Architecture**: Master (roscore) + Nodes + Topics + Services + Actions
✅ **Workspace Structure**: src/ (your code) → build/ (compiled) → devel/ (executables)
✅ **catkin_make** → **source devel/setup.bash** → **rosrun/roslaunch**
✅ **TF**: Coordinate transformations between frames
✅ **Pose**: Position (x,y,z) + Orientation (quaternion)

### MODULE 3: Programming (Key Points)
✅ **Topic**: Publisher (broadcasts) ↔ Subscriber (listens) - continuous data
✅ **Service**: Client (requests) ↔ Server (responds) - one-time action
✅ **Action**: Client (goal) ↔ Server (feedback + result) - long-running tasks
✅ **Custom Messages**: .msg, .srv, .action files in package

### MODULE 4: Sensing & Moving (Key Points)
✅ **Gazebo**: 3D simulator with physics
✅ **URDF**: XML robot description (links + joints)
✅ **Wander-Bot**: Sense obstacles → Turn if close → Move forward
✅ **Teleop-Bot**: Keyboard input → Convert to Twist → Publish to /cmd_vel
✅ **Velocity Ramps**: Gradual acceleration/deceleration for smooth motion

### MODULE 5: Perception & Behavior (Key Points)
✅ **Line Follower Pipeline**: Camera → Image processing → Detect line → Control
✅ **OpenCV**: cv2.cvtColor(), cv2.inRange(), cv2.findContours()
✅ **cv_bridge**: Convert between ROS Image and OpenCV format
✅ **Control**: Error = desired - actual, Angular velocity = Kp × error
✅ **PID**: Proportional + Integral + Derivative for better control

---

## 💡 Top 10 Exam Patterns

### Pattern 1: "Create a publisher"
```python
pub = rospy.Publisher('/topic', MsgType, queue_size=10)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(msg)
    rate.sleep()
```

### Pattern 2: "Create a subscriber"
```python
def callback(msg):
    rospy.loginfo(msg.data)
    
rospy.Subscriber('/topic', MsgType, callback)
rospy.spin()
```

### Pattern 3: "Create a service"
```python
# Server
def handle_request(req):
    result = process(req)
    return ResponseType(result)

rospy.Service('service_name', ServiceType, handle_request)

# Client
rospy.wait_for_service('service_name')
service = rospy.ServiceProxy('service_name', ServiceType)
response = service(request_data)
```

### Pattern 4: "Move robot in a circle"
```python
twist = Twist()
twist.linear.x = 1.0
twist.angular.z = 0.5  # Non-zero for circle
pub.publish(twist)
```

### Pattern 5: "Move robot in a square"
```python
for i in range(4):
    # Forward
    twist.linear.x = 1.0
    twist.angular.z = 0.0
    pub.publish(twist)
    rospy.sleep(2.0)
    
    # Rotate 90°
    twist.linear.x = 0.0
    twist.angular.z = 1.57
    pub.publish(twist)
    rospy.sleep(1.0)
```

### Pattern 6: "Detect obstacle and avoid"
```python
def laser_callback(msg):
    min_dist = min(msg.ranges)
    if min_dist < threshold:
        # Turn
        twist.angular.z = 1.0
    else:
        # Forward
        twist.linear.x = 0.5
    pub.publish(twist)
```

### Pattern 7: "Follow a line"
```python
def image_callback(msg):
    cv_image = bridge.imgmsg_to_cv2(msg)
    line_center = detect_line(cv_image)
    
    error = image_center - line_center
    twist.angular.z = kp * error
    pub.publish(twist)
```

### Pattern 8: "Spawn turtle and control"
```python
# Spawn
spawn = rospy.ServiceProxy('/spawn', Spawn)
spawn(x, y, theta, 'turtle2')

# Control
pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
```

### Pattern 9: "Track current position"
```python
current_pose = None

def pose_callback(msg):
    global current_pose
    current_pose = msg

rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
```

### Pattern 10: "Launch multiple nodes"
```xml
<launch>
    <node pkg="pkg1" type="node1.py" name="node1" output="screen"/>
    <node pkg="pkg2" type="node2.py" name="node2" output="screen"/>
</launch>
```

---

## 🚨 Common Exam Mistakes & Fixes

| Mistake | Fix |
|---------|-----|
| **Node not found** | `source devel/setup.bash` |
| **Script not executable** | `chmod +x script.py` |
| **Wrong topic name** | Use `rostopic list` to check |
| **No roscore** | Start with `roscore` or `roslaunch` |
| **Subscriber not receiving** | Check topic names match exactly |
| **Angles in degrees** | Convert to radians: `deg * π / 180` |
| **Forgot rospy.spin()** | Add it to subscribers |
| **Import errors** | Check message type imports |
| **Queue size missing** | Add `queue_size=10` to publisher |
| **Forgot rate.sleep()** | Publishers need sleep in loop |

---

## 🧠 Memory Tricks

### "PASTA" - ROS Communication Types
- **P**ublish/Subscribe - Topics (continuous)
- **A**ction - Long tasks with feedback
- **S**ervice - Request/Response (one-time)
- **T**ransform (TF) - Coordinate frames
- **A**lways check topic names!

### "BUILD-SOURCE-RUN"
1. **B**uild: `catkin_make`
2. **S**ource: `source devel/setup.bash`
3. **R**un: `rosrun` or `roslaunch`

### "3 Steps to Any Node"
1. **Init**: `rospy.init_node('name')`
2. **Create**: Publisher/Subscriber/Service
3. **Loop**: `while`/`spin()`/`rate.sleep()`

---

## ⏰ 5-Minute Pre-Exam Checklist

**Mental Check**:
- [ ] I know the difference between topic/service/action
- [ ] I can write a publisher from memory
- [ ] I can write a subscriber from memory
- [ ] I know how to make turtle move in patterns
- [ ] I understand the build process (catkin_make → source → run)

**Command Check**:
- [ ] `roscore` - start master
- [ ] `rostopic list` - see topics
- [ ] `rostopic echo /topic` - see messages
- [ ] `rosnode list` - see nodes
- [ ] `catkin_make` - build
- [ ] `source devel/setup.bash` - source
- [ ] `chmod +x script.py` - make executable

**Code Check**:
- [ ] `rospy.init_node()` - start node
- [ ] `rospy.Publisher()` - create publisher
- [ ] `rospy.Subscriber()` - create subscriber
- [ ] `pub.publish(msg)` - send message
- [ ] `rospy.spin()` - keep subscriber alive
- [ ] `rate.sleep()` - control publisher frequency

---

## 🎓 Sample Exam Question Breakdown

**Question**: "Create a robot that follows a wall on its left side using a laser scanner."

### Step 1: Identify Pattern
- **Continuous sensor data** → SUBSCRIBER (laser)
- **Continuous movement** → PUBLISHER (cmd_vel)
- **Real-time behavior** → Regular node (not service)

### Step 2: Sketch Architecture
```
Laser Scanner → Subscriber → Processing → Publisher → Robot Motors
     /scan                   (logic)        /cmd_vel
```

### Step 3: Algorithm
```
1. Subscribe to /scan
2. Read left-side ranges
3. If too close to wall: turn right
4. If too far from wall: turn left
5. Otherwise: move forward
6. Publish to /cmd_vel
```

### Step 4: Code Template
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
        self.desired_distance = 0.5  # 50cm from wall
        
    def callback(self, msg):
        # Get left side distance
        left_distance = msg.ranges[len(msg.ranges)//4]
        
        twist = Twist()
        error = left_distance - self.desired_distance
        
        # Control
        twist.linear.x = 0.3
        twist.angular.z = -error * 2.0  # Negative because left side
        
        self.pub.publish(twist)

if __name__ == '__main__':
    follower = WallFollower()
    rospy.spin()
```

**Time**: 15-20 minutes for this question

---

## 🏆 Final Confidence Boosters

### You Know More Than You Think!

✅ You understand ROS architecture (master, nodes, topics)
✅ You can create publishers and subscribers
✅ You know how to control robot movement
✅ You understand services and when to use them
✅ You can process sensor data
✅ You know how to debug (rostopic, rosnode commands)
✅ You can read and write launch files
✅ You understand the build process

### The Secret: Most Exam Questions Are Variations

- **80%** of questions are publisher/subscriber
- **15%** involve services
- **5%** involve actions

**Master the pub/sub pattern and you're golden!**

---

## 📞 Emergency Hotline (During Exam)

**Problem: "My node isn't starting"**
```bash
# Check 1: Is roscore running?
# Check 2: Did I source?
source ~/catkin_ws/devel/setup.bash
# Check 3: Is it executable?
chmod +x script.py
```

**Problem: "No messages on topic"**
```bash
# Check 1: Is topic correct?
rostopic list
# Check 2: Is publisher running?
rosnode list
# Check 3: See what's published
rostopic echo /topic_name
```

**Problem: "Import error"**
```bash
# Check 1: Did I build?
cd ~/catkin_ws && catkin_make
# Check 2: Did I source after building?
source devel/setup.bash
# Check 3: Is package name correct in CMakeLists.txt?
```

---

## 💪 You've Got This!

**Remember**:
- Stay calm - you know this material
- Read questions carefully - don't rush
- Start with what you know - build confidence
- Use templates - don't reinvent the wheel
- Test as you go - don't wait until the end
- Comment your code - partial credit matters

**The patterns are in your muscle memory now. Trust your training!**

---

# 🌟 ONE MORE THING...

## The Ultimate Exam Weapon: The ROS Mental Model

```
┌─────────────────────────────────────────┐
│  "What does the problem ask for?"       │
└─────────────┬───────────────────────────┘
              │
              ▼
        ┌──────────┐
        │  Data?   │ → Continuous? → TOPIC
        │  Action? │ → One-time? → SERVICE
        │  Task?   │ → Long with feedback? → ACTION
        └──────────┘
              │
              ▼
        ┌──────────┐
        │ Sensor?  │ → SUBSCRIBE
        │ Control? │ → PUBLISH
        │ Both?    │ → BOTH!
        └──────────┘
```

**Trust this decision tree. It will guide you through EVERY question.**

---

## 🎊 Congratulations!

You've completed the ROS Crash Course covering:
- ✅ Module 1: Robot fundamentals (4 hours worth)
- ✅ Module 2: ROS basics (5 hours worth)
- ✅ Module 3: ROS programming (5 hours worth)
- ✅ Module 4: Sensing & moving (4 hours worth)
- ✅ Module 5: Perception & behavior (4 hours worth)
- ✅ **Total: 22+ hours of content in one document!**

**You're prepared. You're ready. Now go ace that midterm!** 🚀

---

**P.S.**: Right before walking into the exam, take 3 deep breaths and remember: You're not memorizing - you're pattern matching. The patterns are already in your brain. Just recognize them and apply the templates.

**You've got this, babygurllll! 💪🎓✨**
