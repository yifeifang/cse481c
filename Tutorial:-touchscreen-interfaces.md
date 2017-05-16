To create a touchscreen interface, you can just create a mobile-friendly web interface.

# Mobile web
A mobile-friendly webpage has this tag in the `<head>` section:
```html
<meta name="viewport" content="width=device-width, initial-scale=1">
```

E.g.,
```html
<html>
  <head>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Robot web app</title>
  </head>
  <body>
  </body>
</html>
```

If you create your [web app with Polymer](https://github.com/cse481sp17/cse481c/wiki/ROS-web-interfaces-with-Polymer), this will already be added to your main page.

# Testing on mobile
A simple way to test how your webpage will work on mobile devices is to use the Chrome Developer Tools.
When viewing a webpage, press `Control+Shift+I`.
This shows the JavaScript console.
From here, click on the mobile device icons in the top left or press `Control+Shift+M`:
![image](https://cloud.githubusercontent.com/assets/1175286/26124715/56158346-3a34-11e7-867d-69fdcfb28282.png)

You can change the screen sizes by selecting a device in the dropdown list.
Each team should have gotten a Nexus 7 tablet.
To add a Nexus 7 to the list of presets, click Edit at the bottom of the list and select "Nexus 7".

![image](https://cloud.githubusercontent.com/assets/1175286/26124873/ec5fc2b2-3a34-11e7-9f98-2b433e4c5acc.png)

# Serving your webpage
You can easily serve your webpage from your lab computer or the robot.
If you are using Polymer, run:
```
polymer serve -H 0.0.0.0
```

If you are not using any special tools, run:
```
python -m SimpleHTTPServer 8081 .
```

In both cases, your webpage will be hosted at the URL: `http://COMPUTERNAME.cs.washington.edu:8081/`

Of course, remember to run the rosbridge server as well:
```
roslaunch rosbridge_server rosbridge_websocket.launch
```

# Something fun
You can try using the [Blinky](https://github.com/hcrlab/blinky) robot face, a web app built with Polymer that is optimized for Nexus 7 screens.
You can use Blinky to display information and to ask the user multiple choice questions.
![blinky](https://cloud.githubusercontent.com/assets/1175286/12600875/baf9204c-c451-11e5-98f5-7fbaa8b57a9e.png)