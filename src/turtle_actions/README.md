# Turtle Actions Package

Complete ROS action implementation with turtlesim navigation.

## Package Contents

- **Action Definition**: `NavigateToGoal.action` - Navigate turtle to (x,y) coordinates
- **Action Server**: `navigate_server.py` - Long-running navigation task
- **Action Client**: `navigate_client.py` - Send goals and receive feedback

## Building

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/turtle_actions/scripts/*.py
```

## Execution

### Terminal 1: Start ROS Master
```bash
roscore
```

### Terminal 2: Start Turtlesim
```bash
source ~/catkin_ws/devel/setup.bash
rosrun turtlesim turtlesim_node
```

### Terminal 3: Run Action Server
```bash
source ~/catkin_ws/devel/setup.bash
rosrun turtle_actions navigate_server.py
```

You should see:
```
[INFO] Navigate To Goal Action Server started!
```

### Terminal 4: Run Action Client
```bash
source ~/catkin_ws/devel/setup.bash
rosrun turtle_actions navigate_client.py
```

## Expected Output

**Client Terminal:**
```
[INFO] Waiting for action server...
[INFO] Action server found!
[INFO] === TEST 1: Go to (8, 8) ===
[INFO] Sending goal: (8.0, 8.0)
[INFO] Distance remaining: 6.32m, Progress: 12.5%
[INFO] Distance remaining: 4.21m, Progress: 35.2%
[INFO] Distance remaining: 2.15m, Progress: 68.9%
[INFO] Distance remaining: 0.54m, Progress: 91.3%
[INFO] SUCCESS! Final distance: 0.0823m
```

**Server Terminal:**
```
[INFO] Navigating to (8.0, 8.0)...
[INFO] Distance: 6.32m, Progress: 12.5%
[INFO] Distance: 4.21m, Progress: 35.2%
[INFO] Distance: 2.15m, Progress: 68.9%
[INFO] Distance: 0.54m, Progress: 91.3%
[INFO] Goal reached!
```

## Key Concepts

### Action Components

1. **Goal**: Target coordinates (target_x, target_y)
2. **Feedback**: Live updates (current_distance, percent_complete)
3. **Result**: Final outcome (success, final_distance)

### Why Actions (Not Services)?

- **Long-running tasks**: Navigation takes time
- **Feedback**: Real-time progress updates
- **Cancellable**: Can preempt goals
- **Asynchronous**: Non-blocking operation

### Code Structure

**Server Pattern:**
```python
import actionlib
from turtle_actions.msg import NavigateToGoalAction

server = actionlib.SimpleActionServer(
    'navigate_to_goal',           # Action name
    NavigateToGoalAction,          # Action message type
    self.execute,                  # Callback function
    False
)

def execute(self, goal):
    # Do work
    feedback = NavigateToGoalFeedback()
    server.publish_feedback(feedback)
    
    # Complete
    result = NavigateToGoalResult()
    server.set_succeeded(result)
```

**Client Pattern:**
```python
import actionlib
from turtle_actions.msg import NavigateToGoalAction, NavigateToGoalGoal

client = actionlib.SimpleActionClient('navigate_to_goal', NavigateToGoalAction)
client.wait_for_server()

goal = NavigateToGoalGoal()
goal.target_x = 8.0
goal.target_y = 8.0

client.send_goal(goal, feedback_cb=my_callback)
client.wait_for_result()
result = client.get_result()
```

## Debugging

### Check if action server is running:
```bash
rostopic list | grep navigate_to_goal
```

Should show:
```
/navigate_to_goal/cancel
/navigate_to_goal/feedback
/navigate_to_goal/goal
/navigate_to_goal/result
/navigate_to_goal/status
```

### Monitor feedback in real-time:
```bash
rostopic echo /navigate_to_goal/feedback
```

### Send manual goal:
```bash
# Install actionlib tools
sudo apt-get install ros-noetic-actionlib-tools

# Send goal
rosrun actionlib_tools axclient.py /navigate_to_goal
```

### Check action messages:
```bash
rosmsg show turtle_actions/NavigateToGoalAction
rosmsg show turtle_actions/NavigateToGoalGoal
rosmsg show turtle_actions/NavigateToGoalFeedback
rosmsg show turtle_actions/NavigateToGoalResult
```

## Troubleshooting

**Error: "No module named 'turtle_actions.msg'"**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

**Error: "Action client not connected to action server"**
- Make sure server is running first
- Check: `rostopic list | grep navigate_to_goal`

**Turtle doesn't move:**
- Check turtlesim is running: `rosnode list | grep turtle`
- Verify publisher: `rostopic echo /turtle1/cmd_vel`

**Permission denied:**
```bash
chmod +x ~/catkin_ws/src/turtle_actions/scripts/*.py
```

## Exam Tips

1. **Action = Long Task**: Use when task takes time and needs feedback
2. **5 Topics Created**: Every action creates 5 topics automatically
3. **SimpleActionServer**: Easiest way to create action server
4. **feedback_cb**: Optional callback to process feedback in client
5. **Preemption**: Actions can be cancelled mid-execution
