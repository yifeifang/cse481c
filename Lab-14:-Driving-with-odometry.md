In previous labs, we have driven the robot by repeatedly sending velocity commands to the base.
However, you will often want to command the robot to drive a certain distance or turn a certain angle.
To do this, you will need to track the robot's position and send velocity commands until it has reached the goal.

# Odometry
To keep track of the robot's position, you can use the `/odom` topic.
Odometry keeps track of where the robot has moved by counting how much each wheel has rotated.
The way to interpret `/odom` is that it tells you where the robot is now relative to where it was when it started keeping track of odometry.
In simulation, the start position is where the robot is when the simulation is started.
On the real robot, the start position is where the robot was when it was last restarted.

In simulation, you can expect odometry to be perfectly accurate.
However, on the real robot, odometry is often inaccurate because 1) its wheel encoders don't have infinite resolution and 2) the wheels may slip.
In future labs, we will incorporate other sensors like the base laser to get a more accurate estimate of the robot's position.

# Interpreting rotations
If you look at the `geometry_msgs/Pose` message, you will notice that it gives you the orientation of the robot as a `geometry_msgs/Quaternion`.

There are four main representations of rotations:
- Fixed-axis angles
- Euler angles
- Rotation matrix
- Quaternion

In this class, we will mostly use the rotation matrix and quaternion representations.

## Fixed-axis / Euler angles
In these representations, you use 3 parameters to express rotations about the X, Y, or Z axes.
In the fixed-axis representation, you rotate about the X/Y/Z axes of a fixed frame.
In the Euler angle representation, the first rotation (e.g., about the X-axis) generates an intermediate frame.
The second rotation is then rotated about an axis (e.g., the Y-axis) of this intermediate frame, which generates a second intermediate frame.
Finally, the third rotation is applied to the second intermediate frame.
In both representations, the order in which you apply these rotations matters.
Additionally, which axes you rotate about can differ (e.g., x-y-z, z-x-z), etc.
The advantage of this representation is that it only uses 3 parameters.

## Rotation matrix
The 3x3 rotation matrix is probably the most useful representation of an orientation, but the downside is that it uses 9 parameters to represent.

Imagine a two coordinate frames, `A` and `B`.
`B` is the same as `A`, but rotated about the Z-axis of A by 30 degrees.
Then note the following:
- The Z-axes of the two frames are the same.
- The X<sub>B</sub>, the unit X-axis `(1, 0, 0)` of `B`, is `(cos(30), sin(30), 0)` in the coordinates of `A`.
- Y<sub>B</sub> is `(-sin(30), cos(30), 0)` in the coordinates of `A`.

![untitled drawing](https://cloud.githubusercontent.com/assets/1175286/25162364/f754fce2-2476-11e7-9b78-8c7699c3a22c.png)

We use a leading superscript to specify which coordinate frame a vector or matrix is expressed in.

The rotation matrix <sup>A</sup>R<sub>B</sub> specifies how frame `B` is rotated relative to frame `A`.
The cool thing that makes rotation matrices so useful is that they are extremely easy to interpret.
If there's only one thing you learn from this entire section, it's the sentence below:

**The columns of a rotation matrix <sup>A</sup>R<sub>B</sub> are the unit X, Y, and Z vectors of `B`, expressed in the coordinate frame of `A`.**

In our example with the frame rotated 30 degrees about the Z-axis, the rotation matrix <sup>A</sup>R<sub>B</sub> is:
```
| cos(30) -sin(30) 0 |
| sin(30)  cos(30) 0 |
| 0        0       1 |
```

Notice that the first column, `cos(30), sin(30), 0`, is the same as <sup>A</sup>X<sub>B</sub>, and notice that the second column is <sup>A</sup>Y<sub>B</sub>.
The third column is `0, 0, 1`, which tells you that the Z-axis of `B` is the same as the Z-axis of `A`.

**Test your knowledge:**
What rotation does this rotation matrix describe?
```
|  0.698, -0.707, 0.111 |
|  0.698,  0.707, 0.111 |
| -0.156,  0,     0.988 |
```

This matrix looks confusing at first, but once you think about it as columns of `B`, it makes more sense.
- The unit X vector of B, `0.698, 0.698, -0.156`, points at a 45 degree angle in quadrant 1, but also points slightly downward.
- The unit Y vector of B, `-0.707, 0.707, 0`, points at a 135 degree angle (quadrant 2) and is in the X-Y plane of `A`.
- The unit Z vector of B, `0.111, 0.111, 0.988`, mostly points up (it is approximately `0, 0, 1`).

Note that the columns of a rotation matrix must be of unit length.

## Quaternions
Quaternions are 4-dimension complex numbers of the form `w + ix + jy + kz`, where *i*, *j*, and *k* are imaginary numbers such that *i<sup>2</sup> = j<sup>2</sup> = k<sup>2</sup> = ijk = -1*.
Quaternions have a number of advantages, including being easy to interpolate, avoiding gimbal lock, and being relatively compact.
However, the main disadvantage is that they are hard to interpret.

One key fact to keep in mind is that the unit quaternion is `w=1, x=0, y=0, z=0`.

## Converting between rotation matrices and quaternions
In Python, use the `tf.transformations` module.
In this module, quaternions are represented as vectors of the form `[x, y, z, w]`.
Matrices are represented as a 4x4 numpy arrays.
If you are not familiar with numpy, look up some tutorials, such as this [quickstart guide](https://docs.scipy.org/doc/numpy-dev/user/quickstart.html).

```
import tf.transformations as tft
help(tft) # Use this to look up documentation.
q = Quaternion(w=1, x=0, y=0, z=0)
tft.quaternion_matrix(

```
