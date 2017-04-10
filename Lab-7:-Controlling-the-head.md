In this lab, you will implement head movement on the Fetch.

# Read the Head Interface docs
The head interface section of the Fetch docs explains that there are two ways to control the head.
The first is to specify a point in space to look at.
The second is to control the pan and tilt angles of the head directly.

# Figure out the joint limits
Look at the "Robot Hardware Overview" section of the Fetch docs.
As we did in the previous labs, we will look up the joint limits and maximum velocity of the pan and tilt angles.

* What are the joint limits of the pan/tilt joints?
* What is the maximum angular speed of the pan/tilt joints?

Based on the joint limits and the maximum angular speed, 2.5 seconds is a reasonable, conservative duration for a head trajectory.

# Implement a demo
In this lab, we will go through the steps of implementing a demo for head movement.

Create a file called `head_demo.py` in `applications/scripts`.

Here is some starter code for the demo file:
```py
#! /usr/bin/env python                                                          
                                                                                
import rospy                                                                    
                                                                                
                                                                                
def print_usage():                                                              
    print 'Usage:'                                                              
    print '    rosrun applications head_demo.py look_at FRAME_ID X Y Z'         
    print '    rosrun applications head_demo.py pan_tilt PAN_ANG TILT_ANG'      
    print 'Examples:'                                                           
    print '    rosrun applications head_demo.py look_at base_link 1 0 1'        
    print '    rosrun applications head_demo.py pan_tilt 0 0.707'               
                                                                                
                                                                                
def wait_for_time():                                                            
    """Wait for simulated time to begin.                                        
    """                                                                         
    while rospy.Time().now().to_sec() == 0:                                     
        pass                                                                    
                                                                                
                                                                                
def main():                                                                     
    rospy.init_node('head_demo')                                                
    wait_for_time()                                                             
    argv = rospy.myargv()                                                       
    if len(argv) < 2:                                                           
        print_usage()                                                           
        return                                                                  
    command = argv[1]                                                           
                                                                                
    if command == 'look_at':                                                    
        if len(argv) < 6:                                                       
            print_usage()                                                       
            return                                                              
        frame_id, x, y, z = argv[2], argv[3], argv[4], argv[5]                  
        rospy.logerr('Not implemented.')                                        
    elif command == 'pan_tilt':                                                 
        if len(argv) < 4:                                                       
            print_usage()                                                       
            return                                                              
        pan, tilt = argv[2], argv[3]                                            
        rospy.logerr('Not implemented.')                                        
    else:                                                                       
        print_usage()                                                           
                                                                                
                                                                                
if __name__ == '__main__':                                                      
    main()
```

As usual, you will need to complete the implementation.

## What's the difference between `applications` and `fetch_api`?
Notice that our repo splits code into library code (`fetch_api`) and application code (`applications`).
A file like `head_demo.py` is an application because it has a `main` function.
In general, we would like our applications to be small, and for them to mostly call out to library code.
A common convention is to place Python application code in a folder named `scripts`, while modules are placed in `src`.

## How do applications know about files in `fetch_api`?
The demo file imports code from the `fetch_api` module.
Normally, in Python, when you import a module, it is pre-installed on the system, or the file by the same name is adjacent to the current file.
So how do our demo files know where `fetch_api` is, when there is no `fetch_api.py` adjacent to our demo files?
This is something that catkin handles.
Our course repo already configures catkin properly, but in the future, we will show you how to configure catkin in this way.

## How do I extend `fetch_api`?
The `fetch_api` module is defined in `fetch_api/src/fetch_api`.
The actual content of the `fetch_api` module is explicitly listed in `fetch_api/src/fetch_api/__init__.py`.
For a class like `Head` to be part of the `fetch_api` module, we must import it in `__init__.py`.

# Implement head movement
Add the following starter code to `fetch_api/src/fetch_api/head.py`:
```py
```

Now, add the `Head` class to the `fetch_api` module by editing `fetch_api/src/fetch_api/__init__.py` and adding the following:
```
from .head import Head
```