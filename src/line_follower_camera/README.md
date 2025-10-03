# Line Follower Camera Package

Line following robot using camera and OpenCV for computer vision.

## Package Contents

- **line_follower.py**: Camera-based line follower with OpenCV

## Dependencies

```bash
sudo apt-get install ros-noetic-cv-bridge python3-opencv
```

## Building

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/line_follower_camera/scripts/*.py
```

## Execution

### Option 1: With Gazebo Simulation

#### Terminal 1: Launch Gazebo World with Line Track
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch gazebo_ros empty_world.launch
```

#### Terminal 2: Spawn Robot with Camera
```bash
# If you have a robot model with camera
roslaunch my_robot spawn.launch
```

#### Terminal 3: Run Line Follower
```bash
source ~/catkin_ws/devel/setup.bash
rosrun line_follower_camera line_follower.py
```

### Option 2: With Real Camera/Webcam

#### Terminal 1: Start ROS Master
```bash
roscore
```

#### Terminal 2: Start USB Camera Node
```bash
sudo apt-get install ros-noetic-usb-cam
rosrun usb_cam usb_cam_node
```

Or remap your camera topic:
```bash
# Find your camera topic
rostopic list | grep image

# Run with topic remapping
rosrun line_follower_camera line_follower.py /camera/rgb/image_raw:=/your_camera_topic
```

#### Terminal 3: Run Line Follower
```bash
source ~/catkin_ws/devel/setup.bash
rosrun line_follower_camera line_follower.py
```

## Expected Output

```
[INFO] Line Follower Started!
[INFO] Line detected at x=320, error=-0.5, angular.z=0.005
[INFO] Line detected at x=315, error=-5.2, angular.z=0.052
[INFO] Line detected at x=321, error=1.3, angular.z=-0.013
[WARN] Line lost - stopping
[WARN] Searching for line...
```

## How It Works

### 1. **Image Acquisition**
```python
self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
```

### 2. **Image Processing Pipeline**
```python
# Convert ROS Image → OpenCV
cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

# Extract ROI (Region of Interest - bottom half)
roi = cv_image[int(height/2):height, 0:width]

# Convert BGR → HSV
hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

# Create mask for black line
mask = cv2.inRange(hsv, lower_threshold, upper_threshold)

# Find contours
contours = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
```

### 3. **Line Detection**
```python
# Find largest contour
largest_contour = max(contours, key=cv2.contourArea)

# Calculate centroid
M = cv2.moments(largest_contour)
cx = int(M['m10'] / M['m00'])  # X-coordinate of centroid
```

### 4. **Proportional Control**
```python
error = cx - width / 2           # How far from center?
twist.linear.x = 0.2             # Constant forward speed
twist.angular.z = -0.01 * error  # Turn based on error
```

## Tuning Parameters

Edit these values in `line_follower.py`:

### Color Thresholds (for different line colors)
```python
# Black line (default)
self.lower_threshold = np.array([0, 0, 0])
self.upper_threshold = np.array([180, 255, 50])

# White line
self.lower_threshold = np.array([0, 0, 200])
self.upper_threshold = np.array([180, 30, 255])

# Red line
self.lower_threshold = np.array([0, 100, 100])
self.upper_threshold = np.array([10, 255, 255])

# Blue line
self.lower_threshold = np.array([100, 100, 100])
self.upper_threshold = np.array([130, 255, 255])
```

### Speed and Control
```python
self.linear_speed = 0.2      # Forward speed (increase for faster)
self.angular_gain = 0.01     # Turn sensitivity (increase for sharper turns)
```

## Debugging

### View camera feed:
```bash
rosrun rqt_image_view rqt_image_view
# Select /camera/rgb/image_raw from dropdown
```

### Check available camera topics:
```bash
rostopic list | grep image
```

### View processed mask (add to code):
```python
# In image_callback, after creating mask:
cv2.imshow("Mask", mask)
cv2.imshow("Original", roi)
cv2.waitKey(1)
```

### Monitor velocity commands:
```bash
rostopic echo /cmd_vel
```

### Test with static image:
```python
# Replace subscriber with file read
cv_image = cv2.imread('test_line.jpg')
```

## Troubleshooting

**Error: "No module named cv2"**
```bash
pip3 install opencv-python
# OR
sudo apt-get install python3-opencv
```

**Error: "No module named cv_bridge"**
```bash
sudo apt-get install ros-noetic-cv-bridge
```

**Robot doesn't follow line:**
1. Check color thresholds match your line color
2. View mask with `cv2.imshow()` to verify detection
3. Increase `angular_gain` for sharper turns
4. Ensure camera is pointing down at line

**No camera topic:**
```bash
# List topics
rostopic list

# Check if camera node is running
rosnode list
```

**Line detection fails:**
- Adjust HSV thresholds
- Improve lighting conditions
- Increase contrast between line and background
- Use `cv2.imshow()` to debug mask

## Key Concepts for Exam

### cv_bridge
Converts between ROS Image messages and OpenCV images:
```python
from cv_bridge import CvBridge
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
```

### Color Spaces
- **BGR**: OpenCV default (Blue-Green-Red)
- **HSV**: Better for color detection (Hue-Saturation-Value)

### Image Processing Flow
1. Subscribe to Image topic
2. Convert ROS Image → OpenCV
3. Apply color filtering (cv2.inRange)
4. Find contours (cv2.findContours)
5. Calculate centroid (cv2.moments)
6. Compute error and control

### Proportional Control
```
error = measured - desired
output = K_p * error
```
Where:
- `error` = distance from line center
- `K_p` = proportional gain (angular_gain)
- `output` = angular velocity

## Advanced: PID Control

For smoother following, replace proportional with PID:

```python
class LineFollower:
    def __init__(self):
        # PID parameters
        self.Kp = 0.01
        self.Ki = 0.0001
        self.Kd = 0.005
        
        self.prev_error = 0
        self.integral = 0
    
    def image_callback(self, msg):
        # ... [image processing] ...
        
        error = cx - width / 2
        
        # PID control
        self.integral += error
        derivative = error - self.prev_error
        
        output = (self.Kp * error + 
                 self.Ki * self.integral + 
                 self.Kd * derivative)
        
        twist.angular.z = -output
        self.prev_error = error
```

## Testing Without Robot

Use `rqt` to test image processing:
```bash
rosrun rqt_gui rqt_gui
# Plugins → Visualization → Image View
```

Or publish test images:
```bash
rosrun image_publisher image_publisher test_track.jpg
```
