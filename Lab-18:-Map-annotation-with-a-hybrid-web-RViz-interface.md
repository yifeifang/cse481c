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

# UserAction
One technique for a web interface to communicate with the server is to treat the user's interaction with the interface as a stream of actions.
In order words, you can publish a message to a topic whenever the user takes some action in the interface (clicks the "Create" button, clicks a "Delete" button, etc.)
In this way, you can test your backend by just 
This stream of actions can be recorded, analyzed, and played back for testing purposes.
