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

## MoveIt
The primary motion planning framework for ROS is called [MoveIt](http://moveit.ros.org/).
You can find more information from:
- The official [MoveIt tutorials](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html) from the MoveIt website.
- The [Fetch manipulation tutorial](http://docs.fetchrobotics.com/manipulation.html)

### `MoveGroup`
MoveIt provides an action, called [MoveGroup](http://docs.ros.org/indigo/api/moveit_msgs/html/action/MoveGroup.html) that actually triggers the motion planning and execution given a goal.
You run the action server as part of the MoveIt backend, `move_group.launch`.
There are many options to set as part of the MoveGroup action, so MoveIt provides Python and C++ classes that wrap the action client and provide convenience methods for you.

In case you are wondering, a "group" is MoveIt's term for a set of joints to plan for.
In the case of the Fetch, there are three groups that have been pre-configured for you: *arm*, *arm_with_torso*, and *gripper*.
The difference between the *arm* and *arm_with_torso* groups is that the *arm_with_torso* will plan to move the torso along with the arm, while the *arm* group can only plan to move the arm.
In this lab, we will use the *arm* group, but using *arm_with_torso* is perfectly reasonable.

### `PlanningSceneInterface`
The MoveGroup action will plan a path that is free of self-collisions, but the arm could still collide with other parts of the environment.
`PlanningSceneInterface` MoveIt's representation of the world.
You use this to specify what the robot should not collide with.
To do this, you "add" primitive shapes or meshes representing the workspace to the planning scene.

MoveIt also has built-in capability to automatically infer the planning scene from sensor data.
However, this is most likely will be very slow, so we recommend using shapes and meshes for now.

# Sending Cartesian goals for the gripper
In this lab, we will get started with the very basics of MoveIt, following the [Fetch manipulation tutorial](http://docs.fetchrobotics.com/manipulation.html#simple-moveit-wave-example).

In `fetch_api/arm.py`, add a method to `Arm` called `move_to_base_pose`:
```py
def move_to_base_pose(self, pose):
    """Moves the end-effector to a pose, using motion planning.

    Args:
        pose: geometry_msgs/Pose, the goal pose for the gripper in the
            Arm.BASE_FRAME frame.

    Returns:
        string describing the error if an error occurred, else None.
    """
    pass
```

To implement this, you will first need to import:
```py
from moveit_msgs.msg import MoveItErrorCodes                                    
from moveit_python import MoveGroupInterface, PlanningSceneInterface
```

Create a `MoveGroup` instance in your `__init__` method:
```py
def __init__(self):
    self._move_group = MoveGroupInterface(ARM_GROUP_NAME, Arm.BASE_FRAME) # "arm" and "base_link"
```

Next, fill out `move_to_base_pose` based on the manipulation tutorial linked above.
- Notice that `self._move_group.get_move_action()` is an ActionClient. So, everything you learned about using ActionClients when you wrote `fetch_api` package applies to this as well.
- Look at the definition of the `MoveGroup` action (linked above). You should see that the `MoveGroupResult` section contains a field called `error`.
- Look up the errors that can occur.
- Your method should return `None` on success, or an error string if the error code of the action result is not `SUCCESS`.

This is a helpful utility function:
```py
def moveit_error_string(val):
    """Returns a string associated with a MoveItErrorCode.
        
    Args:
        val: The val field from moveit_msgs/MoveItErrorCodes.msg
        
    Returns: The string associated with the error value, 'UNKNOWN_ERROR_CODE'
        if the value is invalid.
    """ 
    if val == MoveItErrorCodes.SUCCESS:
        return 'SUCCESS'
    elif val == MoveItErrorCodes.FAILURE:
        return 'FAILURE'
    elif val == MoveItErrorCodes.PLANNING_FAILED:
        return 'PLANNING_FAILED'
    elif val == MoveItErrorCodes.INVALID_MOTION_PLAN:
        return 'INVALID_MOTION_PLAN'
    elif val == MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE:
        return 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE'
    elif val == MoveItErrorCodes.CONTROL_FAILED:
        return 'CONTROL_FAILED'
    elif val == MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA:
        return 'UNABLE_TO_AQUIRE_SENSOR_DATA'
    elif val == MoveItErrorCodes.TIMED_OUT:
        return 'TIMED_OUT'
    elif val == MoveItErrorCodes.PREEMPTED:
        return 'PREEMPTED'
    elif val == MoveItErrorCodes.START_STATE_IN_COLLISION:
        return 'START_STATE_IN_COLLISION'
    elif val == MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS:
        return 'START_STATE_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_IN_COLLISION:
        return 'GOAL_IN_COLLISION'
    elif val == MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS:
        return 'GOAL_VIOLATES_PATH_CONSTRAINTS'
    elif val == MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED:
        return 'GOAL_CONSTRAINTS_VIOLATED'
    elif val == MoveItErrorCodes.INVALID_GROUP_NAME:
        return 'INVALID_GROUP_NAME'
    elif val == MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS:
        return 'INVALID_GOAL_CONSTRAINTS'
    elif val == MoveItErrorCodes.INVALID_ROBOT_STATE:
        return 'INVALID_ROBOT_STATE'
    elif val == MoveItErrorCodes.INVALID_LINK_NAME:
        return 'INVALID_LINK_NAME'                                      
    elif val == MoveItErrorCodes.INVALID_OBJECT_NAME:
        return 'INVALID_OBJECT_NAME'
    elif val == MoveItErrorCodes.FRAME_TRANSFORM_FAILURE:
        return 'FRAME_TRANSFORM_FAILURE'
    elif val == MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE:
        return 'COLLISION_CHECKING_UNAVAILABLE'
    elif val == MoveItErrorCodes.ROBOT_STATE_STALE:
        return 'ROBOT_STATE_STALE'
    elif val == MoveItErrorCodes.SENSOR_INFO_STALE:
        return 'SENSOR_INFO_STALE'
    elif val == MoveItErrorCodes.NO_IK_SOLUTION:
        return 'NO_IK_SOLUTION'
    else:
        return 'UNKNOWN_ERROR_CODE'
```
