In this lab, we will learn to use the pre-built navigation capabilities included with the Fetch robot.

Researchers and engineers have worked on the robot navigation problem for a long time, leading to the development of the [ROS navigation stack](http://wiki.ros.org/navigation).
The Fetch robot uses this stack as well.

# Overview
Navigation can be broken down into several subproblems, including localization, mapping, and path planning.

Localization is the process of determining where the robot is in the world.
Odometry is one way of tracking the robot's position, but, as you might have noticed in the previous labs, it is not very accurate.
A better approach is to fuse information from additional sensors.
The Fetch robot, along with many other robots, has a laser scanner in its base to help with localization.

A common algorithm for localization is called Monte Carlo Localization, also known as a particle filter.
In this algorithm, particles represent a guess at the robot's position.
For each laser scan, the algorithm estimates the probability of each particle seeing that data.
Particles that seem unlikely to have produced that data die off, while more plausible particles are boosted.
[amcl](http://wiki.ros.org/amcl), which stands for Adaptive Monte Carlo Localization, is the most commonly used open-source implementation of this algorithm within the ROS community.

In order to localize the robot, it needs to have a map of the environment.
While floor plans can be used as the map, they can not always be obtained.
Instead, the robot itself can map the environment using its laser scanner.
This is done through a process known as SLAM, or simultaneous localization and mapping.
Some common packages for SLAM in ROS are [karto](http://wiki.ros.org/slam_karto) (used by the Fetch), [gmapping](http://wiki.ros.org/gmapping), and a relatively new system called [cartographer](http://wiki.ros.org/cartographer).
In ROS, maps are often represented using images, which can be edited in image editors to clean up noise.

Finally, the robot needs to be able to plan paths to its destination and avoid obstacles while moving.
This can be accomplished using variants on A* planning.
The Fetch robot also supports "keep out" zones, which specify to the planner that the robot should not go there.
However, the "keep out" map must be separate from the map that is used for localization.
The common ROS navigation stack package for doing planning and obstacle avoidance is [move_base](http://wiki.ros.org/move_base).

For Fetch navigation in particular, you can refer to the [Fetch docs navigation tutorial](http://docs.fetchrobotics.com/navigation.html).

# Building a map
Make sure that the Fetch simulator is running.
Then run:
```
roslaunch fetch_navigation build_map.launch
```

This starts the Karto SLAM node.

## Configure RViz
We will want to have different RViz configs for different purposes.
What to visualize while navigation will be different from what to visualize when working on perception or manipulation.
We can even have multiple instances of RViz open, each with its own configuration.

For building a map, we will want to visualize the following things:
- The robot model
- A grid
- The map
- The laser scan
- An image from the head camera
- (Optional) The point cloud from the depth sensor
- The fixed frame should be set to `map`

Open RViz and configure it to have those displays.
Your RViz window should look like this when done:
![image](https://cloud.githubusercontent.com/assets/1175286/25209079/7d620cf6-252d-11e7-9dfd-1264bd9eb82a.png)

You will also want to commit your RViz config files to your code repository, so that everyone else who uses your code can see the same RViz config.
Typically, we save the RViz config file to a folder named `config`.
Save your config file to `~/catkin_ws/src/cse481c/applications/config/navigation.rviz`.

## Configure a launch file
In this section, we will see how to create a launch file to automate the process of launching multiple things.
In this case, the build a map, we need to run or launch three separate things:
- `roslaunch fetch_navigation build_map.launch`
- `rosrun rviz rviz -d ~/catkin_ws/src/cse481c/applications/config/navigation.rviz`
- `rosrun applications keyboard_teleop.py`

Let's create a launch file in `applications/launch/build_map.launch` that does all three of these at once:
```xml
<launch>
  <include file="$(find fetch_navigation)/launch/build_map.launch" />
  <node pkg="rviz" type="rviz" name="$(anon rviz)" args="-d $(find applications)/config/navigation.rviz" />
  <node pkg="applications" type="keyboard_teleop.py" name="fetch_keyboard_teleop" output="screen" />
</launch>
```

You can read about how to set up launch files in the [roslaunch documentation](http://wiki.ros.org/roslaunch/XML).
The example above illustrates the use of several features.

The first line shows how to launch a different launch file.
Launch files support substitution arguments, such as `$(find PKG_NAME)`, which will be replaced with the directory of a package.

The second line shows how to run RViz with your navigation.rviz config file.
You have to pass in the path to the config file using the -d option.
Note that we use `$(find applications)` instead of hardcoding the location `/home/teamN/catkin_ws/src/cse481c/applications`.
Also note that we give RViz an "anonymous" name using `$(anon rviz)`.
In ROS, if two nodes have the same name, the older node will be shut down.
Anonymous names are guaranteed to be unique, so you can launch this launch file without worrying that it will shut down a different instance of RViz that you might have running.

Finally, the third line shows how to run the keyboard teleop app.
We set `output="screen"` to guarantee that its output will reach standard output, which is generally necessary for applications that read from standard input.

## Do the mapping!
Shut down RViz and the Karto node if they are already running.
With your launch file, you can now run:
```
roslaunch applications build_map.launch
```

With the terminal open next to RViz, you can now drive the robot around and watch it build up a map of the world.
Drive around until your map is complete.

# Sending navigation goals in RViz

## RViz config
Add the following displays to RViz:
- The particles from AMCL
- The global path plan (in blue)
- The local plan (in green)
- The cost map