In your next assignment, you will have to create an interactive marker of a Fetch gripper.
The marker needs to have a clickable menu to go to the gripper pose, open the gripper, or close the gripper.
You also need to change the color of the gripper depending on whether an IK solution was found for that pose.
This lab goes over how to accomplish some of these tasks.

# IKFast
Make sure you have completed the previous lab on getting IKFast working.
Otherwise, your feedback callbacks will take a long time to run.

# General structure
As usual, we recommend wrapping your code in a class:
```py
class GripperTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._im_server = im_server
        # gripper_im = InteractiveMarker() ...
        self._im_server.insert(gripper_im, feedback_cb = self.handle_feedback)

    def start(self):
        pass

    def handle_feedback(self, feedback):
        pass

def main():
    ...
    im_server = InteractiveMarkerServer('gripper_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    rospy.spin()
```

# How to make a gripper marker
You can make a marker out of a mesh.
See [rviz/DisplayTypes/Mesh](http://wiki.ros.org/rviz/DisplayTypes/Marker#Mesh_Resource_.28MESH_RESOURCE.3D10.29_.5B1.1.2B-.5D).

Here are the package URIs for the gripper meshes:
```py
GRIPPER_MESH = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'
```

You will need to create 3 markers: one for the gripper and two for the fingertips.
These markers will be added to a single `InteractiveMarkerControl`, which in turn is added to your `InteractiveMarker`.
See what happens when you 

# What is the end-effector link?
To our eyes, `gripper_link` is the most intuitive end-effector link.
However, MoveIt is actually configured with `wrist_roll_link` as the end-effector.
You can find the offset between `wrist_roll_link` and `gripper_link` using:
```
rosrun tf tf_echo wrist_roll_link gripper_link
```

Initially, your gripper marker, when placed at (0, 0, 0) will look like this:

![image](https://cloud.githubusercontent.com/assets/1175286/25469624/47aa90a6-2ad2-11e7-8908-84b0e07e7701.png)

This is because the meshes are defined in the `gripper_link` frame.
However, you actually want the marker to be centered on the `wrist_roll_link`, like below.
Use the offset above to correct this.

![image](https://cloud.githubusercontent.com/assets/1175286/25469637/63b20d74-2ad2-11e7-9a38-b2defad7c8c6.png)