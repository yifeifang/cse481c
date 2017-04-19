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

# Create a demo that drives to locations on the map
From Lab 13:

To that end, you will extend your script to publish three different InteractiveMarkers at different locations in the room. Clicking on an interactive marker should trigger the robot to move towards that InteractiveMarker. You can make the robot move towards a known location by first rotating towards the target and then moving on a straight line towards it until you are close enough. If another marker is clicked while the robot is moving, it should change course and start moving towards the newly clicked InteractiveMarker.