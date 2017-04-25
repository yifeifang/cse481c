# Check the plan
Look at the [`moveToPose` documentation](http://docs.ros.org/indigo/api/moveit_python/html/classmoveit__python_1_1move__group__interface_1_1MoveGroupInterface.html#a4dec65c1c9de176d51ee8ff66d941894).
It supports a few other arguments, hidden behind `kwargs`.
To find out what those extra arguments are, look at the source code by clicking on the line number, where it says, "Defined on line 161 of move_group_interfacy.py."

You can set the maximum `planning_time` to be longer or shorter, and you can set `plan_only` to True.
If `plan_only` is true, then calling `moveToPose` will compute a motion plan, but not actually execute it on the robot.
This is useful if you have a sequence of poses, and they all need to be reached in order.
If you don't check that all of the poses before you start executing the sequence, you might end up with a failure with the robot's arm halfway through the sequence.

Add a method to `Arm` called `check_base_pose`.
It has the same input and output as `move_to_base_pose`, except it should not cause the robot to move.

Next, use this demo program, `check_cart_pose.py`  to test whether it works.
This tool will tell you whether the gripper can be moved to a certain pose, in the base frame, with the gripper pointing straight ahead (the identity orientation).
```py
#! /usr/bin/env python
                                                                                     
from geometry_msgs.msg import Pose, Point, Quaternion                                
import fetch_api
import rospy
import sys
        
                                                                                     
def wait_for_time():
    """Wait for simulated time to begin.
    """ 
    while rospy.Time().now().to_sec() == 0:
        pass
        
        
def main():
    rospy.init_node('check_cart_pose')
    wait_for_time()                                                                  
    argv = rospy.myargv()
    if len(argv) < 4:
        print 'Usage: rosrun applications check_cart_pose.py X Y Z'                  
    x, y, z = argv[1], argv[2], argv[3]
        
    arm = fetch_api.Arm()
    pose = Pose(Point(x, y, z), Quaternion(0, 0, 0, 1))
    error = arm.check_base_pose(pose)
    if error is None:
        rospy.loginfo('Reachable')
    else:
        rospy.loginfo('Not reachable.') 
    arm.cancel_all_goals()
            
                
if __name__ == '__main__':                                                           
    main()
```

Assuming the torso is at max height, you should see:
```
rosrun applications check_cart_pose.py 0.5 0 1
/check_cart_pose main:28: Reachable
rosrun applications check_cart_pose.py 1 0 1
/check_cart_pose main:30: Not reachable.
```