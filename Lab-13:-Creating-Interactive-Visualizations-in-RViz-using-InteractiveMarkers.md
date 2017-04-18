So far we have treated RViz purely as an output node that visualizes information published by other nodes. Next, we will see how RViz can be used to get input from the user. This is done with the use of [InteractiveMarkers](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started). InteractiveMarkers augment regular Markers by allowing the user to change the position or rotation of the Marker, handle click events on the Marker, or allow them to select something from a context menu assigned to the Marker.

To create an InteractiveMarker you will need to write an InteractiveMarker server that publishes the InteractiveMarker and handles the input given through RViz. To better understand how InteractiveMarkers work, we recommend reading Section 1 of the [InteractiveMarker documentation](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Getting%20Started).

# Creating an InteractiveMarker Server

We will continue to augment the `keyboard_teleop_viz.py` script by adding an InteractiveMarker server to it. Let's start with something basic. First, be sure to import the following.

```
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
```

You will add the following setup code at the beginning of your main function. First, create the server.
```
server = InteractiveMarkerServer("simple_marker")
```

Then, create an InteractiveMarker.
```
int_marker = InteractiveMarker()
int_marker.header.frame_id = "base_link"
int_marker.name = "my_marker"
int_marker.description = "Simple Click Control"
```

Next, create a grey cube Marker for the InteractiveMarker.
```
box_marker = Marker()
box_marker.type = Marker.CUBE
box_marker.scale.x = 0.45
box_marker.scale.y = 0.45
box_marker.scale.z = 0.45
box_marker.color.r = 0.0
box_marker.color.g = 0.5
box_marker.color.b = 0.5
box_marker.color.a = 1.0
```

Next create a MarkerControl, add the Marker to it, and add the control to the InteractiveMarker.

```
button_control = InteractiveMarkerControl()
button_control.interaction_mode = InteractiveMarkerControl.BUTTON
button_control.always_visible = True
button_control.markers.append(box_marker)
int_marker.controls.append( button_control )
```

Next write the callback function that will handle the input received through RViz.

```
def handle_viz_input(input):
    if (input.event_type == InteractiveMarkerFeedback.BUTTON_CLICK):
        rospy.loginfo(input.marker_name + ' was clicked.')
    else:
        rospy.loginfo('Cannot handle this InteractiveMarker event')
```


The final lines of code you need to add in your main function will InteractiveMarker to the server with the callback information.
```
server.insert(int_marker, handle_viz_input)
server.applyChanges()
```

To test the interaction with your InteractiveMarker run your script and run RViz. In RViz you will need to add an InteractiveMarker display and choose the topic to which your server will publish its InteractiveMarker. Now you are all set to test the interaction.

# Triggering Continued Robot Actions though InteractiveMarkers

Different controls for InteractiveMarkers allow obtaining different types of input through RViz. Before going into other types of controls we would like to further explore how to handle simple click inputs and how handling of events can impact the architecture of your code. To that end you will extend your script to publish three different InteractiveMarkers at different locations in the room. Clicking on an interactive marker should trigger the robot to move towards that InteractiveMarker. You can make the robot move towards a known location by servoing, i.e. by moving a little towards it at every step of your loop until you are close enough. During servoing, the keyboard controls should be disabled. If another marker is clicked during servoing, the robot should change course and start servoing towards the newly clicked InteractiveMarker.

