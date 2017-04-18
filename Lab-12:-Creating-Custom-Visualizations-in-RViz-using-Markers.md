In this lab you will create custom visualizations in the RViz 3D display. This is done with the help of [Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker), which are a special type of 3D display you might have noticed in the previous lab. The Markers display allows programmatic addition of various primitive shapes to the RViz 3D view by sending a `visualization_msgs/Marker` or `visualization_msgs/MarkerArray` message.

# Publishing a Marker

Let's start by modifying the `keyboard_teleop.py` script from last week to publish a Marker. Make a copy of this script and rename it `keyboard_teleop_viz.py`. Here is what you will need to import into your script:

```
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
```

Next you will need to create a publisher for `Marker` type messages in your main function, before going into the infinite loop.

```
marker_publisher = rospy.Publisher('visualization_marker', Marker)
```

Before moving on, it would be a good idea to check what the Marker message involves using some of the ROS command line tools that you know very well by now.

Here is a function you can add to your script for publishing a `Marker` of type `Marker.TEXT_VIEW_FACING` which will include the `text` argument passed to the function.

```
def show_text_in_rviz(text):
    marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=0,
                lifetime=rospy.Duration(1.5),
                pose=Pose(Point(0.5, 0.5, 1.45), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.06, 0.06, 0.06),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.0, 1.0, 0.0, 0.8),
                text=text)
    marker_publisher.publish(marker)
```

First, try adding a call to this function from your while loop. Run the script and run RViz simultaneously. Within RViz add a Marker display and specify its topic as `visualization_marker`. You should now see the text in RViz.

Next try modifying the different parameters of the Marker message, re-run the script, and observe the effects on the visualization. You can also experiment with [different types of Markers](http://wiki.ros.org/rviz/DisplayTypes/Marker). Finally try publishing the Marker only once, before entering the infinite loop and observe the effect of the `lifetime` parameter. 

# Visualizing the robot path with Markers

Next you will modify your script to publish a Marker that visualizes the path taken by the robot. To do that you will need to know where the robot is at any given time. To that end, use ROS command line tools to explore the data in the `/odom` or `/odom_combined` topics. You will need to extend your script to subscribe to these messages and update parameters of the path visualization based on the messages received on this topic.

To visualize the path you can use `Marker.LINE_STRIP` or `Marker.SPHERE_LIST` type markers. Rather than adding points to the path with a constant frequency, try adding points only when the robot is displaced by a certain amount from its previous pose. 

When you are done modifying your script you should be able to move the robot around with the the keyboard teleoperation and observe the trace that it leaves behind.
