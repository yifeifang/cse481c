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


class AutoPickTeleop(object):
    def __init__(self, arm, gripper, im_server):
        self._im_server = im_server
        # obj_im = InteractiveMarker() ...
        self._im_server.insert(obj_im, feedback_cb = self.handle_feedback)

    def start(self):
        pass

    def handle_feedback(self, feedback):
        pass


def main():
    ...
    im_server = InteractiveMarkerServer('gripper_im_server')
    teleop = GripperTeleop(arm, gripper, im_server)
    auto_pick = AutoPickTeleop(arm, gripper, im_server)
    teleop.start()
    auto_pick.start()
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
You should create a function that, given a `PoseStamped`, returns either an `InteractiveMarker` or a list of 3 Markers.
See what the marker looks like when you place it at 0, 0, 0, in the `base_link` frame.
You can use the grid lines to get a sense of how the meshes are laid out (each grid square in RViz is 1 meter by 1 meter).
You will need to make some adjustments to the fingertip positions.

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

# Creating 6 DOF controls
Follow the [Interactive Markers: Basic Controls tutorial](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls#Simple_6-DOF_control).
That tutorial is written in C++ and does some unnecessary stuff for illustrative purposes, so try to extract the essence of the tutorial instead of following it directly.
To create the arrows and rings around the marker, you will need to add 6 different controls to your interactive marker, as shown in the tutorial.
Once you are done, the arrows and rings will probably be ginormous, so scale them down using the `scale` field of the `InteractiveMarker`.

You will need to add 6 DOF markers to many grippers, so we recommend creating a function that returns a list of `InteractiveMarkerControl`.
You can then append these controls to an interactive marker:
```py
controls = make_6dof_controls()
interactive_marker.controls.extend(controls)
```

If you are following the C++ tutorial and for some reason, only one of the 6 DOF controls is showing up, then it might be because you are forgetting to use `copy.deepcopy`:
```py
control = InteractiveMarkerControl()
control.name = 'move_x'
controls.append(control)
control.name = 'rotate_x'
controls.append(control)
# Oops, control is the same object in both cases. You need to make a copy with copy.deepcopy(control).
```

# Menu items
An [`InteractiveMarker` msg](http://docs.ros.org/indigo/api/visualization_msgs/html/msg/InteractiveMarker.html) contains a list of `MenuEntry` msgs.
According to the `MenuEntry` documentation, you just assign a non-zero ID to each menu entry, and set its `command_type` to `FEEDBACK`.
Then, in your feedback callback, you can check if the `event_type` is `MENU_SELECT` and if so, use `menu_entry_id` to figure out which menu item was clicked.

Finally, you will need to set the `interaction_mode` of the control that holds your gripper markers to `MENU`.

# Handling drag events
If your gripper is disappearing when you drag the 6 DOF controls, you need to set `always_visible` on the control that holds the gripper.

To check IK on drag events, check if the `event_type` is `POSE_UPDATE` in your feedback callback.

# Changing the gripper color
This is easy.
Just get your `InteractiveMarker`, iterate through the list of markers that comprise your gripper visualization, and change their colors individually.
Then, reinsert your `InteractiveMarker` to your interactive marker server and call `applyChanges()`.

# Assignment videos
Here are some videos of what your assignment might look like.
Keep in mind that these videos only show a simulated robot, but the assignment calls for running your code on the real robot.

Gripper teleop:

[![image](http://i3.ytimg.com/vi/fmbaHcKUPgU/hqdefault.jpg)](https://www.youtube.com/watch?v=fmbaHcKUPgU)

Gripper teleop with automated pick sequence:

[![image](http://i3.ytimg.com/vi/Scc5ph2ZA0s/hqdefault.jpg)](https://www.youtube.com/watch?v=Scc5ph2ZA0s)