In this lab, you will create high-level methods for controlling the robot's linear and angular motion.
Instead of specifying an instantaneous velocity, you will use the robot's odometry information to move the robot a particular distance.

The Fetch robot uses a differential drive base, meaning that it can rotate in place and drive forward and backward.
However, it cannot move sideways very easily.
We will decompose the robot's movement into linear and angular movements.

In both cases, you will need to implement roughly the same strategy:
- Make the `Base` class subscribe to the `odom` topic.
- While the robot has not been moved by the desired amount, send velocity commands to move it in the correct direction.

# Make `Base` subscribe to odometry
First, modify `base.py` so that `Base` subscribes to odometry information.
You can put the subscriber in the `__init__` method and assign a method as a callback like so:
```py
class Base(object):
    def __init__(self):
        self._odom_sub = rospy.Subscriber('odom', nav_msgs.msg.Odometry, callback=self._odom_callback)

    def _odom_callback(self, msg):
        # TODO: do something
```

# Linear movement
To move linearly, add a method to `Base` called `go_forward`:
```py
def go_forward(self, distance, speed=0.1):
    """Moves the robot a certain distance.

    It's recommended that the robot move slowly. If the robot moves too
    quickly, it may overshoot the target. Note also that this method does
    not know if the robot's path is perturbed (e.g., by teleop). It stops
    once the distance traveled is equal to the given distance or more.

    Args:
        distance: The distance, in meters, to move. A positive value
            means forward, negative means backward.
        speed: The speed to travel, in meters/second.
    """
    # TODO: rospy.sleep until the base has received at least one message on /odom
    # TODO: record start position, use Python's copy.deepcopy
    start = copy.deepcopy(LATEST_ODOM)
    rate = rospy.Rate(10)
    # TODO: CONDITION should check if the robot has traveled the desired distance
    # TODO: Be sure to handle the case where the distance is negative!
    while CONDITION:
        # TODO: you will probably need to do some math in this loop to check the CONDITION
        direction = -1 if distance < 0 else 1
        self.move(direction * speed, 0)
        rate.sleep()
```

# Angular movement
Turning is very similar to going forward.
However, you also need to consider what happens if the given angle is greater than 2*pi or less than -2*pi.
You can use Python's modulo operator to deal with this.

Create a method called `turn`:
```py
def turn(self, angular_distance, speed=0.5):
    """Rotates the robot a certain angle.

    Args:
        angular_distance: The angle, in radians, to rotate. A positive
            value rotates counter-clockwise.
        speed: The angular speed to rotate, in radians/second.
    """
    # TODO: rospy.sleep until the base has received at least one message on /odom
    # TODO: record start position, use Python's copy.deepcopy
    start = copy.deepcopy(LATEST_ODOM)
    # TODO: What will you do if angular_distance is greater than 2*pi or less than -2*pi?
    rate = rospy.Rate(10)
    # TODO: CONDITION should check if the robot has rotated the desired amount
    # TODO: Be sure to handle the case where the desired amount is negative!
    while CONDITION:
        # TODO: you will probably need to do some math in this loop to check the CONDITION
        direction = -1 if angular_distance < 0 else 1
        self.move(0, direction * speed)
        rate.sleep()
```

**Hints:**
This part is a bit confusing because it's not as easy to compute the rotation remaining, due to the "wraparound" issue.

Here are some helpful facts:
- `math.atan2` returns values in the range [-pi, pi]
- `x % (2*math.pi)` will always be in the range [0, 2*math.pi]

Let's assume that you can get your current yaw angle and your desired yaw angle in the range [0, 2*math.pi].
How can we compute the remaining angle (assume the remaining angle should be a positive value and we store the direction separately)?
Then, you will need to handle the following 4 cases:
1. Current to goal is counter-clockwise (CCW), no no wraparound in between
1. Current to goal is CCW, with wraparound in between
1. Current to goal is CW, no wraparound in between
1. Current to goal is CW, with wraparound in between

![untitled drawing 3](https://cloud.githubusercontent.com/assets/1175286/25262716/d5c6c4f4-260e-11e7-85b4-6ef781f92892.png)

In the first case, computing the remaining distance to go CCW is easy: goal - current.
However, this doesn't appear to work for the second case: goal - current = -270, but the remaining CCW distance should be 90.
How can you modify this formula to work for both the first and second case?
Once you have this formula, notice that the third and fourth cases are the same as the first two, but with the goal and current positions flipped.

# Test your odometry controllers
You can use this demo, which you should place in `applications/scripts/base_demo.py`:

```py
#! /usr/bin/env python

import math
import fetch_api
import rospy


def wait_for_time():                                                                          
    """Wait for simulated time to begin.
    """                                                                                       
    while rospy.Time().now().to_sec() == 0:                                                   
        pass


def print_usage():                                                                            
    print 'Usage: rosrun applications base_demo.py move 0.1'                                  
    print '       rosrun applications base_demo.py rotate 30'                                 
        
        
def main():
    rospy.init_node('base_demo')
    wait_for_time()
    argv = rospy.myargv()
    if len(argv) < 3:
        print_usage() 
        return
    command = argv[1]
    value = float(argv[2])                                                                    
    
    base = fetch_api.Base()
    if command == 'move':                                                                     
        base.go_forward(value)
    elif command == 'rotate':                                                                 
        base.turn(value * math.pi / 180)                                                      
    else:
        print_usage()


if __name__ == '__main__':
    main()
```

# Create a demo that drives to locations on the map
From Lab 13:

To that end, you will extend your script to publish three different InteractiveMarkers at different locations in the room. Clicking on an interactive marker should trigger the robot to move towards that InteractiveMarker. You can make the robot move towards a known location by first rotating towards the target and then moving on a straight line towards it until you are close enough. If another marker is clicked while the robot is moving, it should change course and start moving towards the newly clicked InteractiveMarker.

**Hints:**
- You will not use `base.go_forward` or `base.turn` directly. Instead, you will merge its internal while loop with an overarching while loop. Hopefully, `go_forward` and `turn` will be useful to you in your actual projects.
- The logic for this task can be a bit complex, here is a suggested outline:

```py
class Driver(object)
    def __init__(self, base):
         self.goal = None
         self._base = base

    def start():
        state = 'turn'
        goal = None
        while True:
            # Check if someone changed self.goal from outside
            if goal != self.goal:
                goal = copy.deepcopy(self.goal)
                # TODO: restart the turn/move sequence
                desired_distance = ??? # Set this to how far the robot should move once pointed in the right direction

            if state == 'turn':
                # TODO: Compute how much we need to turn to face the goal
                if STILL_NEED_TO_TURN:
                    self._base.move(0, DIRECTION * ANGULAR_SPEED)
                else:
                    state = 'move'

            if state == 'move':
                # TODO: Compute how far we have moved and compare that to desired_distance
                # Make sure that the robot has the ability to drive backwards if it overshoots
                if STILL_NEED_TO_MOVE:
                    # TODO: possibly adjust speed to slow down when close to the goal
                    self._base.move(DIRECTION * SPEED, 0)

            rospy.sleep(0.1)
```

We also recommend wrapping the interactive markers in a class, like so:
```py
server = InteractiveMarkerServer('simple_marker')
marker1 = DestinationMarker(server, 2, 2, 'dest1', driver)
marker2 = DestinationMarker(server, 1, 0, 'dest2', driver)
marker3 = DestinationMarker(server, 3, -1, 'dest3', driver)
```

As usual, you can assign a class method as a callback:
```py
class DestinationMarker(object):
    def __init__(self, server, x, y, name, driver):
        # ... Initialization, marker creation, etc. ...
        self._server.insert(int_marker, self._callback)
        self._server.applyChanges()

    def _callback(self, feedback_msg):
        ...
```

The markers can change the goal of the driver like so:
```py
class DestinationMarker(object):
    # ... __init__ and other methods...

    def _callback(self, msg):
         # TODO: How do you get the interactive marker given msg.marker_name?
         # See the InteractiveMarkerServer documentation
         interactive_marker = ???
         position = interactive_marker.pose.position
         rospy.loginfo('User clicked {} at {}, {}, {}'.format(msg.marker_name, position.x, position.y, position.z))
         self._driver.goal = new_position # Updates the Driver's goal.
```

## Accounting for drift
You may notice that the robot will drift off-course.
This is mainly due to imprecision when the robot rotates toward the target.
You can correct for this drift by checking if the robot is pointed close enough to the target at the beginning of your while loop.
If it is too off course, then you can set the state back to "turn."
This way, the robot will continuously keep itself pointed to the target as it drives over.

Another trick that can help is to slow the robot down as it approaches the goal (either for turning or for driving forward).
A simple way to do this is to scale the speed linearly with the remaining distance, but with upper and lower bounds:
```
linear_speed = max(0.05, min(0.5, remaining_distance))
angular_speed = max(0.25, min(1, remaining_angle))
```