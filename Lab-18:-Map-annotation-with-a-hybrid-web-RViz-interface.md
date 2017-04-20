This lab will also guide you through parts of your next assignment.

The goal of this lab is similar to the previous lab: we want an interface for creating poses on a map and sending the robot to them.
However, we want a more user friendly interface than just a command-line interface.
Additionally, it would be nice to annotate poses on the map without having to drive the robot there.

# Overview
We will develop a hybrid web/RViz interface, meaning that we will have both a web browser and RViz open.
They are both frontend interfaces that communicate with our backend node that does the actual work.
The reason why we want to create a hybrid interface is because websites and RViz have different advantages.

| | **Advantages** | **Disadvantages** |
| --- | --- | --- |
| **Web** | Works across all operating systems, including mobile devices. | Too slow to render point clouds, ROS support not mature. |
| **RViz** | Powerful graphics capabilities. | Only runs on Ubuntu Linux Desktop devices. |

In this system, the web interface will implement most of the bookkeeping, while RViz will be used to render the map and the interactive markers.
Specifically:

| **Feature** | **Web** | **RViz** |
| --- | --- | --- |
| **Create pose** | User clicks "Create Pose" button. | User places interactive marker. |
| **List poses** | List of poses displayed in interface. | |
| **Delete pose** | User clicks "Delete" next to pose. | |
| **Send robot to pose** | User clicks "Go here" next to pose. | |
| **Edit pose** (optional) | | User drags interactive marker. |
| **Rename pose** (optional) | User renames in web interface. | |

# Latched topics
"Reactive databases" like [Firebase](https://firebase.google.com/docs/database/) or [Meteor](https://www.meteor.com/) are a hot trend in web development today.
In these systems, changes made to the data are immediately propagated to all other clients viewing the same data and updated in real time, without having to refresh the page or periodically poll the server.
Under the hood, these interfaces are implemented using publish/subscribe systems.
Fortunately, we can implement some of the features of reactive databases using ROS.

One way to simulate a reactive database is to publish/subscribe to a "latched" topic.
You can read in the [rospy Publisher](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers) documentation that with a "latched" topic, the last message sent will be saved and sent to any nodes that subscribe to that topic, even if they started subscribing after the message was published.
In this way, subscribing to this topic is like reading a particular value from a database and subscribing to future updates to that value.
To create a latched topic, add `latch=True` when you create the `Publisher`.

We will publish the list of poses to a latched topic.
You will need to create a msg called `PoseNames`, which is just a list of strings.
Then, your server needs to publish the list of pose names whenever a pose is added, deleted, or renamed.
Finally, your website should subscribe to this topic, and re-render the list of poses whenever it changes.

Note that whenever you create a new message type, you will need to shut down `rosbridge_websocket.launch` and re-source your environment before restarting it:
```
# ... Create new message ...
catkin build
# ... Shut down rosbridge_websocket.launch ...
source ~/.bashrc
roslaunch rosbridge_server rosbridge_websocket.launch
```

# Frontend template
Creating websites with raw Javascript is non-trivial, so we are providing a template for the web frontend again.
You can find it in `cse481c/map_annotator/frontend`.
Copy the `frontend` folder into your own `map_annotator` package.
Once you have it, you should be able to test it out:
```
roslaunch rosbridge_server rosbridge_websocket.launch

# In another terminal
cd ~/catkin_ws/src/cse481c/map_annotator/frontend
python -m SimpleHTTPServer 8080 .
```

Visit localhost:8080 in a web browser and open the JavaScript console.
Now try publishing some latched messages to the `pose_names` topic:
```
rostopic pub /pose_names map_annotator/PoseNames "names:
- 'Test 1'
- 'Test 2'"
```
You should see "Test 1" and "Test 2" appear in the pose list with "Go to" and "Delete" buttons.

![image](https://cloud.githubusercontent.com/assets/1175286/25216800/21d53aee-2559-11e7-8c9c-de15cce503b3.png)

# UserAction
One technique for a web interface to communicate with the server is to treat the user's interaction with the interface as a stream of actions.
In order words, you can publish a message to a topic (e.g., `/user_actions`) whenever the user takes some action in the interface (clicks the "Create" button, clicks a "Delete" button, etc.)
In this way, you can test your backend by just publishing messages to the `/user_actions` topic, even if the frontend isn't finished yet.
This topic can also be recorded, analyzed, and played back for testing purposes.

In this interface, all of the user actions can be specified with a `command` parameter that acts on a pose `name`.
If you want to support renaming a pose from the web interface, you may also want to add an additional parameter, `updated_name`.
Your `UserAction.msg` can look like this:

```
string CREATE=create
string DELETE=delete
string GOTO=goto
# string RENAME=rename
string command
string name # The name of the pose the command applies to
string updated_name # If command is RENAME, this is the new name of the pose
```

At this point, your team should theoretically be able to work in two groups: one group that focuses on developing the rest of the web interface and another that focuses on developing the backend.
To finish the web interface, you will need to instrument the interface such that it publishes the correct UserAction messages in response to button clicks.
To work on the backend, you will need to subscribe the the `/user_actions` topic and add or remove poses as requested by the UserAction.
A third component your team members can work on is the interactive marker interface.

# Interactive marker interface
Once a user creates a marker and gives it a name in the web interface, a new interactive marker should appear in RViz at the ground level at the (0, 0, 0) position and unit orientation.
The marker should be an arrow, so that the user can tell which way the Fetch will be facing in this pose.
The user should be able to change the marker's position and orientation.
As the pose of the marker is changed, the database (or whatever data structure you are using to store the poses) should be updated.
And, if a pose is deleted from the web interface, the marker for that pose should disappear from RViz as well.

**Hint:**
Look at the *Chess Piece* marker in the [Interactive Markers Basic Controls tutorial](http://wiki.ros.org/rviz/Tutorials/Interactive%20Markers%3A%20Basic%20Controls) to get an idea of how to create a marker that can be dragged and rotated in the XY plane.

# Testing with multiple browsers
Once your tool is working, you should be able to load the webpage from your phone by visiting `COMPUTERNAME:8080/` in a web browser, where `COMPUTERNAME` is the name of your lab computer.
If you are not on the CSE-Local wireless network, you may need to append `.cs.washington.edu` to `COMPUTERNAME`.
Deleting a pose on your phone should be reflected on the desktop computer, and vice versa.