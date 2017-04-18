Visualizations of different sensor data and hidden robot states can be extremely helpful in debugging robot software.
[RViz](http://wiki.ros.org/rviz) is a visualization tool in ROS that allows creating visualizations from published messages of certain types.

# Getting started with RViz

Check out the [RViz User Guide](http://wiki.ros.org/rviz/UserGuide) to see everything you can do with RViz.
Let's try a few of its functionalities.

First, fire up the Fetch Gazebo simulator in the "playground" setting. Then open a new terminal and run RViz:

```
rosrun rviz rviz
```

An empty RViz window should appear.
At this point you should see a "Displays" subwindow on the left that includes only the "Global Options," "Global Status," and a "Grid" display.
The dark gray subwindow in the middle is the 3D display.
Different visualizations can be aggregated on the 3D display, as long as they have a common or linked frame of reference. You can ignore the rest of the RViz window for now.
In this lab we will visualize things that are anchored to the `base_link` or `odom` so you should switch the "Fixed Frame" in global options to either of those.

![image](https://cloud.githubusercontent.com/assets/1175286/25155859/309204e4-244c-11e7-97c1-9823cb046567.png)

# Adding Elements to RViz

Next, you will add different visualization elements, called "Displays."
Click on the "Add" button and explore the list of different display types offered in the pop-up.

**Note:** If the display panel does not update or react to clicks, you may need to resize the Rviz window to have it re-render the UI.
This is a known issue.

Start by adding a RobotModel.
A new item should appear in the "Displays" list and a visualization of the Fetch robot should appear in the 3D display. 
Expand the display options to explore the different parameters of the Grid and RobotModel displays.
While the grid visualization does not depend on any data, the RobotModel display is configured by reading the  `robot_description` param.

Please note that RViz is not a simulation of the robot, but rather it is a visualization of the robot that is currently simulated in Gazebo. You can use the robot teleoperation tool you developed last week to change the state of the robot in Gazebo and observe that the change is reflected in RViz. If you use `base_link` as the Fixed Frame you might not notice the base movements, whereas if you use `odom` the robot will get displaced from the center of the grid.

Next, add a LaserScan and a PointCloud2.
In the new displays added to the Displays list on the left, click on the space next to the "Topic" box to reveal the list of potential topics for those displays.
These are the list of topics that include messages of the type that the RViz displays are designed to visualize.
Find the right topics to visualize and configure them so you can differentiate between the two sensor data.
You can use the robot teleoperation tool you developed last week to move the robot’s head around and observe how the visualized point cloud changes.
You can also interact with the 3D display change the perspective and zoom, to view the sensor data differently.

In addition to 3D visualizations, you can add visualizations in separate subwindows within RViz. For example, try adding an Image or a Camera, with different topics that are available.

Now that you get the idea of what RViz is you are welcome to further explore other display types before moving on to creating customized visualizations in the 3D display.

# Saving and loading RViz configurations

You do not want to manually add all these displays every time you use RViz. Instead you should save your current configuration as the default so RViz looks exactly like you have it at the time of saving. Try closing and reopening RViz to make sure you are able to save and reload RViz configurations. You can also save different configuration files that might be suitable for different tasks, and manually or programatically load them when you need them. 

# RViz with the real robot

RViz can similarly be used to visualize the current state of the real physical robot and its sensor data. To try it, open a new window, use the command `setrobot astro` to start communication with the real robot and run RViz.
