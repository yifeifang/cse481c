In [Lab 7](https://github.com/cse481sp17/cse481c/wiki/Lab-7%3A-Controlling-the-arm), we saw how to control the robot's arm by setting the joint angles of the arm.
This is the API that the Fetch provides.
However, it's easier to think about arm movements in terms of X/Y/Z position of the gripper (a.k.a., Cartesian space).

# Overview of some concepts
## Inverse kinematics
Given the joint angles of the arm, it's easy to compute where the gripper (the *end-effector*) will be, since we know how long the arm links are and how they are connected.
This is called the *forward kinematics* of the arm.
The *inverse kinematics* (IK) problem is, given the desired end-effector pose, to find out what the joint angles should be for the end-effector to achieve that pose.
This is a harder problem.
Sometimes, there are multiple arm configurations that result in the same end-effector pose, while other times, the pose is not reachable.
The way to solve the IK problem depends on what kind of arm you have.
For some arms (including that of the Fetch), there is a closed-form solution that makes it relatively easy to solve IK, but this is not true for all arms.

When you use command the robot's gripper to a particular pose, the robot can simply compute the IK solution to figure out the joint angles, and then move the robot's arm using the joint angles.
However, you need to be mindful of the fact that IK solutions sometimes result in weird arm motions that might collide with the environment or flip the gripper upside down (bad if you're holding a tray or a glass of water).
So, sometimes, you want to have complete control over the joint angles.

## Motion planning
To avoid obstacles or ensure that the gripper stays in a particular orientation, you will want to do *motion planning*.
Motion planning generates an arm trajectory for you given the desired goal and any constraints you have (such as the need to avoid obstacles, stay within a workspace, or keep the gripper upright).

# MoveIt
The primary motion planning framework for ROS is called [MoveIt](http://moveit.ros.org/).
The tutorials for MoveIt can be found on the [Tutorials](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html) link of the MoveIt website.

# Sending Cartesian goals for the gripper
