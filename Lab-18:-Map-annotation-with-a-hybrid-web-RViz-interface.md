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

