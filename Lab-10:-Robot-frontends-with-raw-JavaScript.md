In this lab, we will create the web teleop frontend, having made the backend in the previous lab.
If you are not familiar with HTML/CSS/JavaScript, it might be helpful to look up tutorials online, such as from the Mozilla Developer Network ([HTML](https://developer.mozilla.org/en-US/docs/Learn/HTML/Introduction_to_HTML), [CSS](https://developer.mozilla.org/en-US/docs/Learn/CSS/Introduction_to_CSS), [JavaScript](https://developer.mozilla.org/en-US/docs/Learn/JavaScript/First_steps)).

# Create the HTML
We will put all of our frontend code in a folder named `frontend`:
```
cd web_teleop
mkdir frontend
cd frontend
```

First, make sure to download roslibjs and its dependencies:
```
wget http://cdn.robotwebtools.org/EventEmitter2/current/eventemitter2.js
wget http://cdn.robotwebtools.org/roslibjs/current/roslib.js
```

Create your teleop interface webpage in `web_teleop/frontend/index.html`:
```html
<!doctype html>
<html>
  <head>
    <title>Fetch teleop</title>
    <link rel="stylesheet" type="text/css" href="teleop.css" />
    <script src="eventemitter2.js"></script>
    <script src="roslib.js"></script>
    <script src="base.js"></script>
    <script src="torso.js"></script>
    <script src="app.js"></script>
  </head>
  <body onload="init()">
    <h1>Fetch teleop</h1>
    <div id="websocket">
      <span id="websocketStatus"></span><br />
      <label for="websocketUrl">Websocket URL:</label>
      <input id="websocketUrl" type="text"></input>
      <input id="websocketButton" type="button" value="Reconnect"></input>
    </div>
    <div id="base">
      <h2>Base control</h2>
      <table class="base-arrows">
        <tr>
          <td></td><td><span class="arrow-up" id="baseForward">&#8598;</span></td><td></td>
        </tr>
        <tr>
          <td><span class="circle-ccw">&#8634;</span></td>
          <td><span class="arrow-down">&#8598;</span></td>
          <td><span class="circle-cw">&#8635;</span></td>
        </tr>
      </table>
    </div>
    <div id="torso">
      <h2>Torso control</h2>
      Current torso height: <span id="torsoHeight"></span><br />
      <label for="torsoSlider">Set torso height:</label>
      <input id="torsoSlider" type="range" min="0.0" max="0.4" step="0.05"></input>
      <span id="desiredTorsoHeight"></span>
      <input id="torsoButton" type="button" value="Set"></input>
    </div>
  </body>
</html>
```

Add a CSS file in `web_teleop/frontend/teleop.css`:
```css
/* Base control */
.base-arrows {
  font-size: 2em;
}
.base-arrows td {
  cursor: pointer;
}
.base-arrows td:hover {
  background-color: #ddd;
}
.base-arrows span {
  display: block;
}
.arrow-down {
  -ms-transform: rotate(225deg);
  -webkit-transform: rotate(225deg);
  transform: rotate(225deg);
}
.arrow-up {
  -ms-transform: rotate(45deg);
  -webkit-transform: rotate(45deg);
  transform: rotate(45deg);
}
.circle-ccw {
  -ms-transform: rotate(-90deg);
  -webkit-transform: rotate(-90deg);
  transform: rotate(-90deg);
}
.circle-cw {
  -ms-transform: rotate(90deg);
  -webkit-transform: rotate(90deg);
  transform: rotate(90deg);
}

/* Torso */
#desiredTorsoHeight {
  font-weight: bold;
}
```

Now start a web server:
```
cd frontend
python -m SimpleHTTPServer 8080 .
```

And visit http://localhost:8080 in a web browser.

# An approach to organizing JavaScript
JavaScript is different from languages like Java and C++ in that uses a "prototype" based inheritance system.
There are also fairly intricate rules for the "this" keyword.
Here, we will describe one approach for organizing JavaScript code into something that resembles Java or C++.

```js
Foo = function() {
  this.publicVariable = 10;
  var privateVariable = 5;
  this.increment = function(x) {
    privateVariable += x;
  };
  this.show = function() {
    console.log(this.publicVariable, privateVariable);
  };
}

var foo = Foo();
foo.show(); // 10 5
foo.publicVariable = 11;
foo.show(); // 11 5
foo.privateVariable = 10; // Error, no such thing as privateVariable (was not added to "this")
foo.increment(1);
foo.show(); // 11 6
```

## that and this
The value of the `this` keyword can change depending on context in JavaScript and the rules are often hard to remember.
In most callbacks and event listeners, `this` will not refer to the instance of the object you are writing.
As a result, we often assign `this` to a local variable named `that` in the main function body.
`that` will be captured in the function closures for the callbacks and will always refer to the object (e.g., `Foo`).

If you are not sure what `this` refers to, try printing it out by using `console.log(this)`.
Press Ctrl+Shift+J in Chrome to see the JavaScript console.
If `this` does not refer to the object you are writing (e.g., `Foo`), then you might want to write something like this:

```js
Foo = function() {
  var button = document.querySelector('#button');
  this.publicVariable = 10;
  var privateVariable = 5;
  var that = this;
  button.addEventListener('click', function() {
    console.log(this); // "this" does not refer to Foo!
    console.log(that); // "that" refers to this instance of Foo.
    that.publicVariable += privateVariable;
  });
}
```

# Create the main app file
`web_teleop/frontend/app.js`:
```js
App = function() {
  // HTML elements
  var websocketStatus = document.querySelector('#websocketStatus');
  var websocketUrl = document.querySelector('#websocketUrl');
  var websocketButton = document.querySelector('#websocketButton');

  // Compute websocket URL.
  var url = (function() {
    var hostname = window.location.hostname;
    var protocol = 'ws:';
    if (window.location.protocol == 'https:') {
      protocol = 'wss:'
    }
    return protocol + '//' + hostname + ':9090';
  })();
  websocketUrl.value = url;

  // This is a common technique in JavaScript callbacks.
  // If you are not sure what 'this' refers to (and the rules are often unclear,
  // just assign a local variable (named 'that') to 'this' outside the callback
  // and use that variable instead.
  var that = this;

  // Connects to the websocket URL and sets this.ros.
  this.connect = function(url) {
    this.ros = new ROSLIB.Ros({url: url});
    this.ros.on('connection', function() {
      websocketStatus.textContent = 'Connected to websocket server.';
      if (that.base) {
        that.base.stop();
      }
      that.base = new Base(that.ros);
      that.torso = new Torso(that.ros);
    });
    this.ros.on('error', function(error) {
      websocketStatus.textContent = 'Error connecting to websocket server.';
    });
    this.ros.on('close', function() {
      websocketStatus.textContent = 'Disconnected from websocket server.';
    });
  }

  // Set up the "Reconnect" button.
  var connectFromButton = function() { that.connect(websocketUrl.value); };
  websocketButton.addEventListener('click', connectFromButton);

  // Initialize app.
  this.connect(url);
};

// init is called in index.html at <body onload="init()">
function init() {
  var app = new App();
}
```

# Add the base controller
`web_teleop/frontend/base.js`:
```js
Base = function(ros) {
  // HTML elements
  // To get an element with an ID of "baseForward", query it as shown below.
  // Note that any IDs you set on HTML elements should be unique.
  var baseForward = document.querySelector('#baseForward');

  var that = this;

  // Public variables (outsiders can set this using base.linearSpeed = 0.1)
  this.linearSpeed = 0.25;
  this.angularSpeed = 0.25;

  // Set up the publisher.
  var cmdVel = new ROSLIB.Topic({
    ros: ros,
    name: '/cmd_vel',
    messageType: 'geometry_msgs/Twist'
  });

  // Internal function to send a velocity command.
  var move = function(linear, angular) {
    var twist = new ROSLIB.Message({
      linear: {
        x: linear,
        y: 0,
        z: 0
      },
      angular: {
        x: 0,
        y: 0,
        z: angular
      }
    });  
    cmdVel.publish(twist);
  }

  // Handler for when the mouse is held on the up arrow.
  // Instead of writing a loop (which will block the web page), we use
  // setInterval, which repeatedly calls the given function at a given
  // time interval. In this case, it repeatedly calls move() every 50 ms.
  // Note that inside of move, we use that._timer and that.linearSpeed.
  // At the top of the file we set "var that = this" to ensure that the
  // local variable "that" always refers to this Base instance.
  this.moveForward = function() {
    that._timer = setInterval(function() {
      move(that.linearSpeed, 0)
    }, 50);
  }

  // Stops the robot from moving.
  this.stop = function() {
    if (that._timer) {
      clearInterval(that._timer);
    }
    move(0, 0);
  };  

  baseForward.addEventListener('mousedown', that.moveForward);
  
  // We bind stop() to whenever the mouse is lifted up anywhere on the webpage
  // for safety reasons. We want to be conservative about sending movement commands.
  document.addEventListener('mouseup', that.stop);
}
```

# Add the torso controller
`web_teleop/frontend/torso.js`:
```js
Torso = function(ros) {
  // HTML elements
  var torsoHeight = document.querySelector('#torsoHeight');
  var desiredTorsoHeight = document.querySelector('#desiredTorsoHeight');
  var torsoSlider = document.querySelector('#torsoSlider');
  var torsoButton = document.querySelector('#torsoButton');

  var that = this;

  var setTorsoClient = new ROSLIB.Service({
    ros: ros,
    name: '/web_teleop/set_torso',
    serviceType: 'web_teleop/SetTorso'
  });

  // Listen to torso height from the joint_state_republisher.
  var listener = new ROSLIB.Topic({
    ros: ros,
    name: 'joint_state_republisher/torso_lift_joint',
    messageType: 'std_msgs/Float64'
  });

  listener.subscribe(function(message) {
    // Whenever we get a message with a new torso height, update
    // the torso height display on the webpage.
    var height = message.data;

    // Note the noise in the data. You can smooth it out using this line of code.
    // height = Math.round(height*1000) / 1000
    torsoHeight.textContent = height;
  });
  
  // Initialize slider.
  var desiredHeight = 0.1;
  desiredTorsoHeight.textContent = desiredHeight;
  // For most input elements, the .value field is both a getter and a setter.
  // Here we can set its value to the default (0.1).
  torsoSlider.value = desiredHeight;

  // Update desiredHeight when slider moves.
  torsoSlider.addEventListener('input', function() {
    // Read where the slider is now.
    desiredHeight = torsoSlider.value;
    // Update the desired torso height display.
    desiredTorsoHeight.textContent = desiredHeight;
  });

  // Method to set the height.
  this.setHeight = function(height) {
    var height = Math.min(Math.max(0.0, height), 0.4);
    var request = new ROSLIB.ServiceRequest({
      height: height
    });
    setTorsoClient.callService(request);
  };

  // Set the height when the button is clicked.
  torsoButton.addEventListener('click', function() {
    that.setHeight(desiredHeight);
  });
}
```

![image](https://cloud.githubusercontent.com/assets/1175286/24991283/91d75fd8-1fce-11e7-8292-982a51123984.png)

# Add an image display
You are supposed to be able to use this app to control the robot without looking at Gazebo.
But clearly, without looking at Gazebo, we are blind!
To fix this, we must embed an image display.

To do this, install the Web Video Server (also from Robot Web Tools):
```
sudo apt-get install ros-indigo-web-video-server
```

Then, add it to your `backend.launch`:
```xml
<node pkg="web_video_server" type="web_video_server" name="web_video_server">
  <param name="port" value="8000" />
</node>
```
Here we tell the web video server to run on port 8000, since we are using port 8080 for our webpage.

Run your backend again and visit http://localhost:8000/stream_viewer?topic=/head_camera/rgb/image_raw.
You should see the view from the robot's head-mounted camera.

Now, add an image to your `index.html`:
```html
<div id="camera">
  <img src="//localhost:8000/stream?topic=/head_camera/rgb/image_raw"></img>
</div>
```

Refresh the page and you should see the following:
![image](https://cloud.githubusercontent.com/assets/1175286/24991988/8407f746-1fd3-11e7-802d-9b17db52fb8d.png)

Hopefully you now have an idea of how to complete the teleop assignment.
You will need to add:
- The remaining base movement commands
- Commands to move the head
- Commands to move the arm to a pose
- You can put the real robot in a desired pose and read the joint values using your `joint_state_reader`.
- Commands to open/close the gripper