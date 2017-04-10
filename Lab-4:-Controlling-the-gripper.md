# actionlib
To control the gripper (and most other interfaces of the robot), you will need to be familiar with [actionlib](http://wiki.ros.org/actionlib) and the [actionlib tutorials](http://wiki.ros.org/actionlib/Tutorials).

Do the actionlib beginner tutorials if you have not already.

# Gripper interface
Go to the [Fetch docs](http://docs.fetchrobotics.com/index.html) and click on *API Overview* -> *Gripper Interface*.
The robot runs an actionlib server to control the gripper, so you will need to write an actionlib client to send open/close commands.

# Write the action client
In the course repo, a skeleton of the gripper code has been written for you in `fetch_api/src/fetch_api/gripper.py`.
Fill in the methods `open` and `close`.

# Finish the demo
The course repo contains a partially implemented file you can use to test your code in `applications/scripts/gripper_demo.py`.
Fill out the sections that simply print "Not implemented."

If you run the demo with:
```
rosrun applications gripper_demo.py
```

You should see the error:
```
[rosrun] Couldn't find executable named gripper_demo.py below /home/teamN/catkin_ws/src/cse481c/applications
[rosrun] Found the following, but they're either not files,
[rosrun] or not executable:
[rosrun]   /home/teamN/catkin_ws/src/cse481c/applications/scripts/gripper_demo.py
```

Any ROS Python file with a main function should be marked executable:
```
chmod +x gripper_demo.py
```

Now you should be able to run the demo, and see the robot's gripper open and close in the simulator:
```
rosrun applications gripper_demo.py close
rosrun applications gripper_demo.py open
```