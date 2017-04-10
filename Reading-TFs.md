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
You can visualize the structure of the frame graph using `view_frames`:
```
rosrun tf view_frames
xdg-open frames.pdf
rm frames.gv frames.pdf # Clean up
```

To actually see a visualization of the frames, open `rviz`:
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

# Reading TFs from code
To read the transform between two frames, follow the TF tutorials on writing a TF listener.
You will need to know how to do this for later labs.