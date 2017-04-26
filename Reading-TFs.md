[TF](http://wiki.ros.org/tf) is an important framework for world modeling in ROS.
In TF, we can attach coordinate frames to locations of interest, which can include parts of the robot's body and parts of the world.
A frame is represented as a position and orientation offset (called a *transform*) relative to another frame.
Knowing a set of transforms between frames, TF can construct the overall frame graph and find the transform between any two frames.
In TF, the frame graph is restricted to be a tree.

For example, suppose we have the following frames:
* The center of the room (named *map*)
* The base of the room (named *base_link*)
* The robot's gripper (named *gripper_link*)

And we know the following transforms:
* *map* -> *base_link* (from the robot's odometry)
* *base_link* -> *gripper_link* (from the robot's internal sensors)

Then TF can compute the transform between *map* -> *gripper_link*.

We recommend going through the [tf tutorials](http://wiki.ros.org/tf/Tutorials).

# Visualizing frames
You can visualize the structure of the frame graph using `rqt_tf_tree`:
```
rosrun rqt_tf_tree rqt_tf_tree
```
This shows the TF tree after accumulating TF messages for 5 seconds from the time you run it.
It will not update live if the TF tree changes.

To see a visualization of the frames, open `rviz`:
```
rosrun rviz rviz
```

Switch the "Fixed Frame" to `base_link`:
![image](https://cloud.githubusercontent.com/assets/1175286/24841887/800800c4-1d43-11e7-9ed2-2e1bffd18e83.png)

Click "Add" in the bottom left of the window to add a display and double-click on "TF":
![image](https://cloud.githubusercontent.com/assets/1175286/24841988/1dbaada2-1d45-11e7-9b47-e26f3923a411.png)

Add a "Robot model" display as well.
You will see all the coordinate frames attached to the robot:
![image](https://cloud.githubusercontent.com/assets/1175286/24842000/4aa9726c-1d45-11e7-806f-a5722fa80de3.png)

This visualization is too busy, so expand the TF display options in the left sidebar and uncheck all the frames by unchecking "Frames->All enabled". Then check frames one by one (e.g., `base_link`, `gripper_link`).

In these visualizations, red is the x-axis, green is the y-axis, and blue is the z-axis.
A simple way to remember this is RGB = XYZ.

You can also visualize a single frame using the "Axes" display.

# Reading TFs from code
To read the transform between two frames, follow the TF tutorials on writing a TF listener.

## Time-related issues with TF
As with the `joint_states` topic, many nodes can publish a subset of the TF tree to the `tf` topic.
A `TransformListener` works by accumulating messages on this topic and building a tree.
A consequence of this is that using a `TransformListener` immediately after creating it might not work, since it has not had time to receive any callbacks on the `tf` topic.
If you run into this issue, you should wait a little bit after creating your `TransformListener` with `rospy.sleep(0.1)`.

Another common issue is to run into errors like "Transform would require extrapolation into the past/future."
TF not only keeps track of the transforms between frames, but also the exact moment in time when that transform was true.
It stores a short history of this data in a buffer (about 10 seconds long).
In this way, you can ask TF, what is the transform between where the gripper is now and where it was 1 second ago?
Extrapolation errors mean that the transform is being requested for a time that's outside of the buffer.
If you run into this issue, either:
- Put the transform lookup in a try-catch and retry a few times before giving up
- Make sure that you're supplying correct timestamps (often specified in the header field of most messages)

## End-effector reader
Create a demo file called `ee_pose_demo.py`.
This demo should continuously output the pose of the gripper in the base_link frame.