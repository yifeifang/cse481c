In [Lab 14](https://github.com/cse481sp17/cse481c/wiki/Lab-14%3A-Odometry-and-rotations), we discussed how to interpret rotations.
This lab will expand on that and explain *transformations*, which combine rotation and translation.
This will be useful for computing pre-grasp poses and other waypoints.

# Transformations
A transformation is a combination of a rotation and translation.
As discussed in the previous lab, the notation <sup>A</sup>T<sub>B</sub> means that T describes frame B in terms of frame A.
A 3D rotation matrix <sup>A</sup>R<sub>B</sub> is 3x3, and the columns of the matrix are the standard basis (i.e., the unit X, Y, and Z vectors) of the frame B in terms of frame A.
This assumes that the two frames have the same origin.

A *homogeneous transform matrix* <sup>A</sup>T<sub>B</sub> is 4x4 and describes both the rotation and the translation offset of frame B in terms of frame A.
The rotation matrix is embedded in the upper left hand corner of T.
The translation vector is embedded in the upper right hand corner of T.
The bottom row of a homogeneous transform matrix is always 0, 0, 0, 1.

The translation vector describes the origin of frame B in terms of frame A.

**Example:**
Here is a homogeneous transform matrix <sup>A</sup>T<sub>B</sub>.
What does frame B look like?
```
| cos(45) -sin(45) 0  0   |
| sin(45)  cos(45) 0  0   |
| 0        0       1  0.5 |
| 0        0       0  1   |
```

*Answer:*
The rotation matrix is in the upper left corner.
The rotation matrix tells us that if frame B had the same origin as frame A, frame B would be the same as frame A, but rotated by 45 degrees around the Z axis.
However, B does not have the same origin as A. B's origin, in A's coordinate system, is (0, 0, 0.5).
So B is rotated by 45 degrees and is raised up by 0.5 compared to A.

**Example 2:**
With your right hand, point your index finger forward, your middle finger to the left, and your thumb up.
Your index finger is the X axis, your middle finger is the Y axis, and your thumb is the Z axis.

Now, roll your hand so that your thumb is pointing left and your middle finger is pointing down.
Next, move your hand 10 centimeters closer to you.

What is the homogeneous transform matrix describing your hand's new pose (frame B) in terms of its first pose (frame A)?

*Hints:*
- Remember that the columns of the rotation matrix describing B in terms of A are the unit X, Y, and Z vectors of B in A's coordinate system.

# Pose = transformation
You have worked with `geometry_msgs/Pose` in previous labs, and you know that it has a `position` and `orientation` field.
Because a transformation is a position and orientation, a `geometry_msgs/Pose` can be thought of as a transformation.
However, you often need to specify what this transformation is relative to.
`PoseStamped` messages contain a `header` field, which contains a `frame_id`.
The `frame_id` is equivalent to "frame A" in the examples above.

