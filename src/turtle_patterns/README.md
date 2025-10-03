# 🐢 Turtle Patterns Package

Complete turtlesim movement patterns with go-to-goal functionality.

## 📦 Installation

Already created! Just build:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 🚀 How to Run

### Step 1: Start turtlesim
```bash
roscore
```

In a new terminal:
```bash
rosrun turtlesim turtlesim_node
```

### Step 2: Run any pattern

**Circle:**
```bash
rosrun turtle_patterns move_circle.py
```

**Square:**
```bash
rosrun turtle_patterns move_square.py
```

**Spiral:**
```bash
rosrun turtle_patterns move_spiral.py
```

**Figure-8:**
```bash
rosrun turtle_patterns move_figure8.py
```

**Diamond:**
```bash
rosrun turtle_patterns move_diamond.py
```

**Go to specific point:**
```bash
# Go to (8, 8)
rosrun turtle_patterns go_to_goal.py 8 8

# Go to (5, 5)
rosrun turtle_patterns go_to_goal.py 5 5
```

## 📝 Pattern Details

| Pattern | Description | Linear Speed | Angular Speed |
|---------|-------------|--------------|---------------|
| Circle | Constant radius | 2.0 | 1.0 |
| Square | 4 sides + 90° turns | 2.0 | π/2 |
| Spiral | Expanding radius | radius * 2.0 | 2.0 |
| Figure-8 | Two circles opposite | 2.0 | ±1.0 |
| Diamond | 45° rotated square | 2.0 | π/4 then π/2 |
| Go-to-Goal | Navigate to (x,y) | Proportional | Proportional |

## 🔧 Troubleshooting

**Turtle not moving?**
```bash
# Check if node is running
rosnode list

# Check if messages are being published
rostopic echo /turtle1/cmd_vel
```

**Permission denied?**
```bash
chmod +x ~/catkin_ws/src/turtle_patterns/scripts/*.py
```

## 📚 Code Structure

All scripts follow this pattern:
1. Initialize node
2. Create publisher to `/turtle1/cmd_vel`
3. Create Twist message
4. Set linear.x (forward/back) and angular.z (rotation)
5. Publish continuously

**Key Concept:** Circle = linear.x + angular.z both non-zero
