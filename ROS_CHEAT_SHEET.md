# 📋 ROS EXAM CHEAT SHEET - PRINT THIS!

---

## 🔥 MOST USED CODE BLOCKS

### Publisher Template (Copy-Paste)
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

rospy.init_node('pub_node')
pub = rospy.Publisher('/topic', String, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    msg = String()
    msg.data = "data"
    pub.publish(msg)
    rate.sleep()
```

### Subscriber Template (Copy-Paste)
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(msg.data)

rospy.init_node('sub_node')
rospy.Subscriber('/topic', String, callback)
rospy.spin()
```

### Movement Template (Copy-Paste)
```python
#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

rospy.init_node('move_node')
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

twist = Twist()
twist.linear.x = 1.0   # forward
twist.angular.z = 0.5  # turn

while not rospy.is_shutdown():
    pub.publish(twist)
    rate.sleep()
```

---

## 🎯 MOVEMENT PATTERNS

| Pattern | linear.x | angular.z | Time |
|---------|----------|-----------|------|
| **Straight** | 1.0 | 0.0 | - |
| **Circle** | 1.0 | 0.5 | - |
| **Spiral** | r * w | w | r += 0.01 |
| **Square** | 1.0 → 0.0 | 0.0 → π/2 | 2s → 1s × 4 |
| **Stop** | 0.0 | 0.0 | - |

---

## 📦 COMMON IMPORTS

```python
import rospy
from std_msgs.msg import String, Int32, Float32
from geometry_msgs.msg import Twist, Pose, Point
from sensor_msgs.msg import LaserScan, Image
from turtlesim.msg import Pose as TurtlePose
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
```

---

## 🔧 BUILD & RUN

```bash
# 1. Create package
cd ~/catkin_ws/src
catkin_create_pkg my_pkg rospy std_msgs geometry_msgs

# 2. Build
cd ~/catkin_ws
catkin_make

# 3. Source
source devel/setup.bash

# 4. Make executable
chmod +x src/my_pkg/scripts/script.py

# 5. Run
rosrun my_pkg script.py
```

---

## 🐛 DEBUG COMMANDS

```bash
# Essential commands
rostopic list           # See all topics
rostopic echo /topic    # See messages
rosnode list            # See all nodes
rosnode kill /node      # Kill a node

# Publishing manually
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 1.0}"

# Info
rostopic info /topic
rosnode info /node
rosmsg show geometry_msgs/Twist
```

---

## 🚨 QUICK FIXES

| Problem | Solution |
|---------|----------|
| Node not found | `source devel/setup.bash` |
| Permission denied | `chmod +x script.py` |
| No messages | Check topic name with `rostopic list` |
| Import error | Add to package.xml dependencies |

---

## 📝 SERVICE QUICK CODE

**Server:**
```python
#!/usr/bin/env python3
import rospy
from my_pkg.srv import AddTwoInts, AddTwoIntsResponse

def handle(req):
    return AddTwoIntsResponse(req.a + req.b)

rospy.init_node('server')
rospy.Service('add', AddTwoInts, handle)
rospy.spin()
```

**Client:**
```python
#!/usr/bin/env python3
import rospy
from my_pkg.srv import AddTwoInts

rospy.init_node('client')
rospy.wait_for_service('add')
add = rospy.ServiceProxy('add', AddTwoInts)
result = add(10, 20)
print(result.sum)
```

---

## 🚀 TURTLESIM QUICK

```python
# Spawn turtle
from turtlesim.srv import Spawn
spawn = rospy.ServiceProxy('/spawn', Spawn)
spawn(5.0, 5.0, 0.0, 'turtle2')

# Move turtle
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
twist = Twist()
twist.linear.x = 2.0
pub.publish(twist)

# Get position
from turtlesim.msg import Pose
def callback(msg):
    print(msg.x, msg.y, msg.theta)
rospy.Subscriber('/turtle1/pose', Pose, callback)
```

---

## 📷 LINE FOLLOWER QUICK

```python
#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

def callback(msg):
    img = bridge.imgmsg_to_cv2(msg, "bgr8")
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    lower = np.array([20, 100, 100])
    upper = np.array([30, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    
    M = cv2.moments(mask)
    if M['m00'] > 0:
        cx = int(M['m10'] / M['m00'])
        error = (img.shape[1] // 2) - cx
        
        twist = Twist()
        twist.linear.x = 0.2
        twist.angular.z = 0.005 * error
        pub.publish(twist)

rospy.init_node('line_follower')
rospy.Subscriber('/camera/image_raw', Image, callback)
rospy.spin()
```

---

## 🎯 EXAM DECISION TREE

```
Question Type → Code Type
├─ "publish data" → Publisher
├─ "receive data" → Subscriber
├─ "call on demand" → Service
├─ "move robot" → Twist Publisher to /cmd_vel
├─ "avoid obstacle" → LaserScan Subscriber
└─ "follow line" → Image Subscriber + Control
```

---

## 💡 PRO TIPS

1. **Always start with a template** - don't code from scratch
2. **Test as you code** - use `rostopic echo` frequently
3. **Topic names must match exactly** - check with `rostopic list`
4. **Remember to source** - `source devel/setup.bash`
5. **Angles in radians** - π = 3.14159, π/2 = 1.5708

---

## 🔢 COMMON VALUES

```python
import math

# Angles
math.pi         # 180 degrees
math.pi / 2     # 90 degrees
math.pi / 4     # 45 degrees

# Speeds (typical)
linear.x = 0.2 to 2.0    # m/s
angular.z = 0.5 to 2.0   # rad/s

# Rates
rate = rospy.Rate(10)    # 10 Hz most common
```

---

## ⚡ FASTEST WAY TO PASS

**Master these 3 patterns:**

1. **Publisher** (50% of exam)
```python
pub = rospy.Publisher('/topic', Type, queue_size=10)
pub.publish(msg)
```

2. **Subscriber** (30% of exam)
```python
def callback(msg):
    # do something
rospy.Subscriber('/topic', Type, callback)
rospy.spin()
```

3. **Movement** (20% of exam)
```python
twist = Twist()
twist.linear.x = 1.0
twist.angular.z = 0.5
pub.publish(twist)
```

**Know these = Pass exam!** ✅

---

## 🎓 LAST MINUTE CHECKLIST

Before exam:
- [ ] I can write a publisher
- [ ] I can write a subscriber
- [ ] I know how to move robot (Twist)
- [ ] I know `rostopic list` and `rostopic echo`
- [ ] I remember to `source devel/setup.bash`
- [ ] I remember to `chmod +x` scripts

---

**PRINT THIS PAGE AND KEEP IT WITH YOU!**

**Good luck! You got this! 💪🚀**
