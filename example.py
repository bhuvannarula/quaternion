from quaternion import *

'''
A Point/Frame undergoes Transformation to get new Point/Frame.

So, we start with a Point.
Point (derived from) DualQuaternion class
'''
origin = Point(
    Rotation(),
    Translation([0, 0, 0])
)


'''
We define a 'Transformation' as an operation/dual quaternion that works on a Point to transform it.

So, every Link is a Transformation.
Transformation (derived from) DualQuaternion class

Transformation*Transformation -> Transformation
'''
# The parameters for defining Link are its D-H Parameters.
link1 = Link([pi/3, 7, 0, pi/2])
link2 = Link([pi/6, 0, 10, 0])
link3 = Link([pi*(-40/180), 0, 10, 0])


'''
Now, if we multiply Point with Transformation, we basically transform a Point through the Transformation.

So,
Point*Transformation -> Point
'''
end = origin*link1*link2*link3


'''
Point has two defined functions,

Point.position() -> Quaternion
Returns Position in Cartesian Coordinates.

Point.orientation() -> Quaternion
Returns Orientation in Quaternion Form.
'''
end_xyz = end.position()
end_orient = end.orientation()

# Now, we can print to see the end-point.
print(end_xyz)

'''
Output: [0 x y z]
[[ 0.        ]
 [ 9.25416578]
 [16.02868532]
 [10.26351822]]
'''