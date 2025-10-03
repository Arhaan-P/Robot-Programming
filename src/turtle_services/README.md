# 🔧 Turtle Services Package

Complete ROS service examples with turtlesim integration.

## 📦 Installation

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 🚀 How to Run

### Example 1: Add Two Integers Service

**Terminal 1: Start Server**
```bash
roscore
```

**Terminal 2: Run Server**
```bash
rosrun turtle_services add_server.py
```

**Terminal 3: Call Service**
```bash
rosrun turtle_services add_client.py 10 20
# Output: Result: 10 + 20 = 30
```

### Example 2: Go To Point Service (Turtlesim)

**Terminal 1: Start turtlesim**
```bash
rosrun turtlesim turtlesim_node
```

**Terminal 2: Start Go-To-Point Server**
```bash
rosrun turtle_services goto_server.py
```

**Terminal 3: Call Service**
```bash
# Go to point (8, 8)
rosrun turtle_services goto_client.py 8 8

# Go to point (2, 2)
rosrun turtle_services goto_client.py 2 2
```

## 📋 Service Definitions

### AddTwoInts.srv
```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

### GoToPoint.srv
```
# Request
float64 x
float64 y
---
# Response
bool success
string message
```

## 🔍 Debug Commands

```bash
# List all services
rosservice list

# Call service from command line
rosservice call /add_two_ints 10 20

# Get service info
rosservice info /add_two_ints

# Show service type
rosservice type /add_two_ints
```

## 🔧 Troubleshooting

**Service not found?**
```bash
# Make sure server is running
rosnode list

# Check if service exists
rosservice list
```

**Permission denied?**
```bash
chmod +x ~/catkin_ws/src/turtle_services/scripts/*.py
```

**Import error?**
```bash
# Rebuild workspace
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 📚 Key Concepts

- **Service** = Request-Response pattern (one-time call)
- **Server** = Waits for requests, processes them, returns response
- **Client** = Sends request, waits for response
- **Use case**: On-demand actions (not continuous data like topics)
