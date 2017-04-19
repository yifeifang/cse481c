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

# Sending navigation goals in RViz