In the labs, you used raw JavaScript with Robot Web Tools, which can be hard to program with as your application becomes more complex.
You can use Robot Web Tools with a web framework to make your life easier.
In this tutorial, we will show how to use the [Polymer](https://www.polymer-project.org) web framework with ROS.
In particular, we will rewrite the map annotator web interface with Polymer.

# Overview

The high-level concept of Polymer is that it helps you create your own HTML elements.
Elements can have their own data and methods.

Once you create an element, you can add this element to your web pages or to other elements.
Similarly, your element can use other elements created by the community.

We have created a set of [ROS Polymer elements](https://www.webcomponents.org/collection/jstnhuang/ros-element-collection) that you can use.

Polymer is on the verge of releasing a version 2.0, which has breaking changes compared to Polymer 1.0.
We will use Polymer 1.0 in this tutorial, which work with our ROS Polymer elements.

# Install Polymer
We have installed Node.js on the course computers.
You may need to install Bower and Polymer:
```
npm install -g bower polymer-cli
```

# Create a project

We have already installed Polymer on the course computers.
A project can be created in any folder, but for ROS projects you might want to put it in a `frontend` folder in your package.

```
cd ~/catkin_ws/src/cse481c/map_annotator
mkdir frontend2 # Since we already have a frontend folder
cd frontend2
polymer init
```

You will see this menu:
```
? Which starter template would you like to use? 
❯ polymer-1-element - A simple Polymer 1.0 element template 
  polymer-2-element - A simple Polymer 2.0 element template 
  polymer-1-application - A simple Polymer 1.0 application template 
  polymer-2-application - A simple Polymer 2.0 application 
  polymer-1-starter-kit - A Polymer 1.x starter application template, with navigation and "PRPL pattern" loading 
  polymer-2-starter-kit - A Polymer 2.x starter application template, with navigation and "PRPL pattern" loading 
  shop - The "Shop" Progressive Web App demo
```

Select `polymer-1-application`.
Name the application `map-annotator` and leave the main element name and app description to their defaults.
```
? Which starter template would you like to use? polymer-1-application
info:    Running template polymer-1-application...
? Application name map-annotator
? Main element name (map-annotator-app)
? Brief description of the application
```

# Run your app
Run:
```
polymer serve
```

If you are working remotely, you can tell Polymer to open up the website to the public internet by running:
```
polymer serve -H 0.0.0.0
```

Then, you can visit `localhost:8081` in your web browser to see the application so far.

# Install the ROS elements
In this lab, you will only need the `ros-topic` element.
However, you can also use `ros-service` and `ros-actionclient` in a similar way (see the element collection).

To use elements that others have written, you install them using the `bower` tool.
To download an element hosted on Github, e.g., at `https://github.com/jstnhuang/ros-topic`, run `bower install --save jstnhuang/ros-topic`.
You need to run `bower install` in the root of the project directory, where `bower.json` is.

```
~/catkin_ws/src/cse481c/map_annotator/frontend2
bower install --save jstnhuang/ros-topic
bower install --save jstnhuang/ros-websocket
```

Now the `ros-topic` element is in the `bower_components/` folder.
`bower_components/` contains a ton of files, which should be downloaded via bower and not stored in Git.
```
echo "bower_components/" >> .gitignore
```

# Run your backend
Run your map annotator backend as described in the previous labs.

# Edit your frontend code
Edit `src/map-annotator-app/map-annotator-app.html`
This is your "main" element.
Normally, you break your app up into smaller elements.
We will break this map annotator app into two elements, the main element and an element representing a single map location.

## Connect to the websocket server
The main page should connect to the websocket server.
This can be done using the `<ros-websocket>` element.
When you downloaded `<ros-topic>` with bower, bower automatically downloaded `<ros-websocket>` as a dependency.

To find the documentation for the `<ros-*>` elements, look them up on webcomponents.org:
- [`<ros-websocket>`](https://www.webcomponents.org/element/jstnhuang/ros-websocket)
- [`<ros-topic>`](https://www.webcomponents.org/element/jstnhuang/ros-topic)

Then click on "API" in the left navigation menu.

Add the following line to the top of `map-annotator-app.html`:
```html
<link rel="import" href="../../bower_components/ros-websocket/ros-websocket.html">
```

Then add this to the HTML section:
```html
<ros-websocket auto
  ros="{{ros}}"
  on-connection="handleConnection"
  on-close="handleClose"
  on-error="handleError">
</ros-websocket>

<h2>Hello [[prop1]] (change this to a title of your choosing)</h2>
<div>
  Websocket status: {{status}}
</div>
```

Here, we set `<ros-websocket>` to automatically connect to the default URL of `ws://localhost:9090`.
It exposes a `ros` variable that other ROS Polymer elements need.
When it connects or fails to connect to the websocket server, it fires events (`connection`, `close`, or `error`).
We specify that certain callbacks should be called when it fires those events.

In our HTML, we display a variable called `status` to show the connection status.
We will update this variable in our connection callbacks.
Add the following handler methods to the JavaScript section:
```js
properties: {
  status: {
    type: String,
    value: 'Unknown',
  },
},

handleConnection: function() {
  this.status = 'Connected to the websocket server.';
},

handleClose: function() {
  this.status = 'Closed connection to websocket server.';
},

handleError: function() {
  this.status = 'Error connecting to websocket server.';
},
```

Make sure you are running `roslaunch rosbridge_server rosbridge_websocket.launch`, and refresh the page.
You should see the status say "Connected to the websocket server."
If you shut down the server, you should see the status change to "Closed connection to websocket server."

## Subscribe to the pose list
Now, let's subscribe to the pose list.

The [ros-topic documentation](https://www.webcomponents.org/element/jstnhuang/ros-topic/ros-topic) shows how to subscribe to a topic.

First, import the `<ros-topic>` element:
```html
<link rel="import" href="../../bower_components/ros-topic/ros-topic.html">
```

Now, add the element to the HTML section:
```html
<ros-websocket ...>
</ros-websocket>
<ros-topic
  auto
  last-message="{{poseList}}"
  on-message="handlePoseList"
  topic="/pose_names"
  ros="{{ros}}"
  msg-type="map_annotator/PoseNames"
></ros-topic>
```

The `last-message` property of `<ros-topic>` binds the most recently received message on the topic to the variable `poseList`.
If a new message is published to the topic, then the `poseList` variable will be automatically updated.

To programmatically access the messages, we can define an event handler for the `message` event.
We define the handler in the JavaScript section:
```js
handlePoseList: function(evt) {
  var msg = evt.detail;
  console.log(msg);
}
```

Refresh your web page.
To see messages printed from `console.log`, open the JavaScript console in your web browser.
If you have the map annotator backend running, you should see a message appear in the JavaScript console with the list of pose names.
You can also publish a list of names using `rostopic pub`.

## Display the pose list
Polymer can render a list of items like so:

HTML:
```html
<h3>Poses</h2>
<template is="dom-repeat" items="{{poseList.names}}">
  <div>{{item}}</div>
</template>
```

Refresh the page and you should see a list of names.

# Add an "Add pose" button
In this section, we'll see how to use `<ros-topic>` to send messages on a topic.
To do this, we'll implement the functionality to add a new map pose.

The create button will be a [`<paper-button>`](https://www.webcomponents.org/element/PolymerElements/paper-button), which is a button with [Material Design](https://material.io/) styling.

Download it if you don't already have it (you can find these commands on the documentation page if you click "Installed via bower").
```
bower install --save PolymerElements/paper-button
```

Now import it into `map-annotator-app.html`:
```html
<link rel="import" href="../../bower_components/paper-button/paper-button.html">
```

Add it to your HTML:
```html
<h3>Poses</h2>                             
<paper-button on-tap="handleAdd">Add pose</paper-button>
```

Now add a callback for when the button is pressed:
```js
handleAdd: function() {
  var name = alert('Enter a name:');
  console.log(name);
},
```

## Add some styles
Update the style section:
```html
<style>
  :host {
    display: block;
    font-family: Sans-serif
  }
  paper-button {
    background-color: #ccc;
    margin-bottom: 10px;
    text-transform: none;
  }
</style>
```

## Publish the UserAction
Now, let's see how to publish the `UserAction` to create a pose.
First, we will add another `<ros-topic>` element to the page:
```html
<ros-topic
  auto
  id="userActions"
  topic="/user_actions"
  ros="{{ros}}"
  msg-type="map_annotator/UserAction"
></ros-topic>
```

We have added an `id` property to this `<ros-topic>` so that we can easily refer to it in our JavaScript code.
Update the `handleAdd` function:
```js
handleAdd: function() {
  var name = prompt('Enter a name:');
  var msg = {
    command: 'create',
    name: name,
    updated_name: ''
  };     
  this.$.userActions.publish(msg);
},
```

If you assign a unique `id` (in this case, `userActions`) to an element in your HTML, then you can access it in the JavaScript section using `this.$.userActions`.

Refresh your page.
Now, click on "Add pose" should pop up a prompt and publish a `UserAction` message.
Check in RViz that your interface is working, or inspect the `/user_actions` topic using `rostopic echo /user_actions`.

# Create a new element
Now, let's see how to break up the application into smaller pieces.
Instead of implementing the list items in this "main" element, let's create a new element that represents a single list item.

```
cd frontend2/src
mkdir map-annotator-pose
cd map-annotator-pose
```

Create a new file, `map-annotator-pose/map-annotator-pose.html`, and add this boilerplate:
```html
<link rel="import" href="../../bower_components/polymer/polymer.html">
<link rel="import" href="../../bower_components/paper-button/paper-button.html">
<link rel="import" href="../../bower_components/ros-topic/ros-topic.html">
  
<dom-module id="map-annotator-pose">
  <template>
    <style>
      :host {
        display: block;
      } 
      paper-button {
        background-color: #ccc;
        margin-bottom: 10px;
        text-transform: none;
      }
    </style>
    {{name}}
  </template>

  <script>
    Polymer({
      is: 'map-annotator-pose',

      properties: {
      },
    });
  </script>
</dom-module>
```

Here, we have already imported `<paper-button>` and `<ros-topic>` for you.
We also copied over the styling for the button.

A `<map-annotator-pose>` will have a `name` property, which is a String:
```js
properties: {
  name: {
    type: String,
    value: ''
  },
},
```

## Using `<map-annotator-pose>`
Go back to editing `map-annotator-app.html`.

Import your new `<map-annotator-pose>` element:
```html
<link rel="import" href="../map-annotator-pose/map-annotator-pose.html">
```

In the HTML block that loops through the list items, use a `<map-annotator-pose>` instead of just displaying the name:
```html
<template is="dom-repeat" items="{{poseList.names}}">
  <map-annotator-pose name="{{item}}"></map-annotator-pose>
</template>
```

If you did this correctly, then your app should not look any different when you reload the page.

## Add UserActions to `<map-annotator-pose>`
The last thing we need to do to finish up the app is to add "Go to" and "Delete" buttons to each pose in the list.

To do this, we just follow the same steps that we did when we implemented the "Add pose" button in the main app.

First, add the `/user_actions` publisher to the page:
```html
<ros-topic
  auto
  id="userActions"
  topic="/user_actions"
  ros="{{ros}}"
  msg-type="map_annotator/UserAction"
></ros-topic>
```

Notice that this requires a `ros` variable to be passed into it.
The `ros` variable was supplied by `<ros-websocket>` in the main element, so we need to pass it in as a property.
Add `ros` to the `<map-annotator-pose>` `properties` object:
```js
properties: {
  name: {
    type: String,
    value: ''
  },
  ros: Object
},
```

`ros: Object` is just shorthand for a property that doesn't have a default value.

We will have to pass in the ROS object back in the `<map-annotator-app>` as well:
```html
<template is="dom-repeat" items="{{poseList.names}}">
  <map-annotator-pose ros="{{ros}}" name="{{item}}"></map-annotator-pose>
</template>
```

Back in the `<map-annotator-pose>` element, add two buttons and click handlers for both of them:
```html
{{name}}
<paper-button on-tap="handleGoTo">Go to</paper-button>
<paper-button on-tap="handleDelete">Delete</paper-button>
```

And define the click handlers in the JavaScript section:
```js
handleGoTo: function() {
  var msg = {
    command: 'goto',
    name: this.name,
    updated_name: ''
  };
  this.$.userActions.publish(msg);
},

handleDelete: function() {
  var msg = {
    command: 'delete',
    name: this.name,
    updated_name: ''
  };
  this.$.userActions.publish(msg);
},
```

# Final thoughts
As you can see, Polymer makes it easier to build complex web applications because you can break down your code into individual elements.
The fact that it has reactive data binding makes it a perfect fit for publish/subscribe systems like ROS, as well.
For example, we were able to create an auto-updating list without writing any JavaScript code:
```html
<ros-topic
  ...
  last-message="{{poseList}}"
></ros-topic>
<template is="dom-repeat" items="{{poseList.names}}">
  <map-annotator-pose ros="{{ros}}" name="{{item}}"></map-annotator-pose>
</template>
```

However, the more advanced usages of Polymer do have a steep learning curve, and its documentation is not very beginner-friendly.
If you have another web framework that you are familiar with, you should feel free to try and use Robot Web Tools within that framework instead.
If you would like to learn more about Polymer, you can read:
- The [Polymer tutorials](https://www.polymer-project.org/1.0/start/)
- The [Polymer documentation](https://www.polymer-project.org/1.0/docs/devguide/feature-overview)

In case it wasn't clear where the code snippets above went, here is the full code.

**map-annotator-app.html:**
```html
<link rel="import" href="../../bower_components/polymer/polymer.html">
<link rel="import" href="../../bower_components/paper-button/paper-button.html">
<link rel="import" href="../../bower_components/ros-topic/ros-topic.html">
<link rel="import" href="../../bower_components/ros-websocket/ros-websocket.html">
<link rel="import" href="../map-annotator-pose/map-annotator-pose.html">

<dom-module id="map-annotator-app">
  <template>
    <style>
      :host {
        display: block;
        font-family: Sans-serif
      }
      paper-button {
        background-color: #ccc;
        margin-bottom: 10px;
        text-transform: none;
      }
    </style>
    <ros-websocket auto
      ros="{{ros}}"
      on-connection="handleConnection"
      on-close="handleClose"
      on-error="handleError">
    </ros-websocket>
    <ros-topic
      auto
      last-message="{{poseList}}"
      on-message="handlePoseList"
      topic="/pose_names"
      ros="{{ros}}"
      msg-type="map_annotator/PoseNames"
    ></ros-topic>
    <ros-topic
      auto
      id="userActions"
      topic="/user_actions"
      ros="{{ros}}"
      msg-type="map_annotator/UserAction"
    ></ros-topic>
    <h2>Map annotator</h2>
    <div>
      Websocket status: {{status}}
    </div>
    <h3>Poses</h2>
    <paper-button on-tap="handleAdd">Add pose</paper-button>
    <template is="dom-repeat" items="{{poseList.names}}">
      <map-annotator-pose ros="{{ros}}" name="{{item}}"></map-annotator-pose>
    </template>
  </template>

  <script>
    Polymer({

      is: 'map-annotator-app',

      properties: {
        status: {
          type: String,
          value: 'Unknown.',
        },
      },

      handleConnection: function() {
        this.status = 'Connected to websocket server.';
      },

      handleClose: function() {
        this.status = 'Closed connection to websocket server.';
      },

      handleError: function() {
        this.status = 'Error connecting to websocket server.';
      },

      handlePoseList: function(evt) {
        var msg = evt.detail;
        console.log(msg);
      },

      handleAdd: function() {
        var name = prompt('Enter a name:');
        var msg = {
          command: 'create',
          name: name,
          updated_name: ''
        };
        this.$.userActions.publish(msg);
      },

    });
  </script>
</dom-module>
```

**map-annotator-pose.html:**
```html
<link rel="import" href="../../bower_components/polymer/polymer.html">
<link rel="import" href="../../bower_components/paper-button/paper-button.html">
<link rel="import" href="../../bower_components/ros-topic/ros-topic.html">

<dom-module id="map-annotator-pose">
  <template>
    <style>
      :host {
        display: block;
      }
      paper-button {
        background-color: #ccc;
        margin-bottom: 10px;
        text-transform: none;
      }
    </style>
    <ros-topic
      auto
      id="userActions"
      topic="/user_actions"
      ros="{{ros}}"
      msg-type="map_annotator/UserAction"
    ></ros-topic>
    {{name}}
    <paper-button on-tap="handleGoTo">Go to</paper-button>
    <paper-button on-tap="handleDelete">Delete</paper-button>
  </template>

  <script>
    Polymer({

      is: 'map-annotator-pose',

      properties: {
        name: {
          type: String,
          value: ''
        },
        ros: Object,
      },

      handleGoTo: function() {
        var msg = {
          command: 'goto',
          name: this.name,
          updated_name: ''
        };
        this.$.userActions.publish(msg);
      },

      handleDelete: function() {
        var msg = {
          command: 'delete',
          name: this.name,
          updated_name: ''
        };
        this.$.userActions.publish(msg);
      },
    });
  </script>
</dom-module>
```