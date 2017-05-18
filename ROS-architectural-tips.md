Here are some considerations and tips for designing the architecture of your system.

# Nodes
It is a good idea to split your code into multiple nodes to do different tasks, rather than put everything into one giant node.
The advantages of splitting your code include:
- It's easy to divide the work between team members
- It reduces the chance of conflicts in Git
- It makes your code easier to understand
- Different nodes can be written in different languages
- If a node crashes, you can restart the faulty node without restarting your whole system

On a technical note, having multiple nodes adds a form of parallelism to your system.
This is because each node is run in its own process.

# Launch files
Launch files are a useful tool for starting up multiple nodes at the same time.
While you could open 10 terminal windows with tmux, this leaves you prone to forgetting which nodes have been restarted, what parameters were set, etc.

You should read the [launch file documentation](http://wiki.ros.org/roslaunch/XML) and see what options are available to you.
Here are a few useful tips.

## RViz
If you store config files in your Git repository, the substitution arg `$(find PACKAGENAME)` expands to the directory of a package.
For example, if you store an RViz config file in `/home/teamN/catkin_ws/src/cse481c/grocerybot/config/grocery.rviz`, you can run RViz with that config file by passing in the `-d` option:
```xml
<node name="$(anon rviz)" pkg="rviz" type="rviz" args="-d $(find grocerybot)/config/grocery.rviz" />
```

## respawn and required
If you have a stateless node that might crash once in a while, you can configure that node to automatically restart by setting `respawn="true"` in the `<node>` tag.
```xml
<node name="perception_server" pkg="grocerybot" type="perception_server" respawn="true" />
```

Suppose you have a node that is critical to your system (for example, your main node).
Rather than hobble along silently if that node crashes, you might want to "fail fast" so that you can debug your system.
In that case, you can set `required="true"` on a `<node>`, which shuts down everything in the launch file if that one node fails.
```xml
<node name="grocerybot_main" pkg="grocerybot" type="main.py" required="true" />
```

# Topics vs. services vs. actions
You will need to use either topics, services, or actions to communicate between nodes.

Topics are good when multiple nodes need to know the same information.
Topics should be used for "fire and forget" use cases, where the publisher doesn't care about getting a response back.
A third consideration is that subscribers should be able to process the data faster than the data is being published.
If not, then some of the messages will be dropped.

Latched topics send the most recently published message to any subscribers that join the topic at a later time.
This is useful for storing state that other nodes might need to "catch up" on.
For example, our map annotator node published the list of poses to a latched topic.
This allowed web browsers to subscribe to the topic and get the list of poses.
While it's possible to implement a service that returns a list of poses, there's no way to know if the pose list changed unless you periodically call the service to check for updates.

Services are exactly like a function call.
You call the service with some input and get some output in response.
One disadvantage of services is that when you call a service, your program will block until the service call returns.
As a result, services are best used for fast operations, or for operations in which there is literally nothing else productive for the robot to do while it waits.
```py
long_service_call1(req1) # Blocks for 10 seconds
long_service_call2(req1) # Blocks for 10 seconds
# You just waited for 20 seconds
```

Actions are like services, but better suited for long-running operations.
The downside is that actions are the most complicated to implement.
If you implement an action server, a client can "start" an action and do other things while the action runs.
You can also receive periodic feedback messages if you want to know about the progress of the system.

```py
long_action_client1.send_goal(goal1) # Starts an action that takes 10 seconds
long_action_client2.send_goal(goal2) # Starts an action that takes 10 seconds
# The two actions run in parallel
long_action_client1.wait_for_result()
long_action_client2.wait_for_result()
# You just waited for 10 seconds
```

# Statelessness
You might want to think about which nodes are *stateless*, meaning that it doesn't hold state in-memory that is important to the functioning of your system.
In other words, a stateless node can be killed and restarted without affecting your system.
For example, the map annotator system from Lab 18 should have been stateless.
Although it held important state (the list of poses), it simply read the list of poses from a pickle file on startup.
If, for example, the map annotator server didn't save the poses to disk, then it would be a *stateful* server.
Restarting that server would be a disaster because you would lose the pose list.

Stateless nodes are, in general, better than stateful nodes because you can restart a stateless node without affecting the system.
However, you can't avoid having at least one stateful node.

# Main node
If you have one stateful node, it should be the "main" node.
The main node contains the outer loop of your system and controls the logic and flow of your system.
Because it is a critical part of your system, you want the main node to be resilient and never crash.
To do this, you will want to avoid doing "real" work in your node and just delegate to other nodes.
Then, you can use try/catch statements and do other error handling to make sure that the node never crashes.

Even if something bad does happen, your main node should simply direct the UI to say something like "Sorry, I'm currently out of service."
This will maintain the best possible customer experience.