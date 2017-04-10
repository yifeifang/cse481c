# actionlib
To control the gripper (and most other interfaces of the robot), you will need to be familiar with [actionlib](http://wiki.ros.org/actionlib) and the [actionlib tutorials](http://wiki.ros.org/actionlib/Tutorials).

Do the actionlib tutorials if you have not already.

# Gripper interface
Go to the [Fetch docs](http://docs.fetchrobotics.com/index.html) and click on *API Overview* -> *Gripper Interface*.
The robot runs an actionlib server to control the gripper, so you will need to write an actionlib client to send open/close commands.

# Write the action client
