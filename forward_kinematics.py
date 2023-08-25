from quaternion import *

origin = Point(
    Rotation(),
    Translation([0, 0, 0])
)

def forward(joint_states : list):
    ang = joint_states

    base_link = Link([ang[0], 7, 0, pi/2])
    link1 = Link([ang[1], 0, 10, 0])
    link2 = Link([ang[2], 0, 10, 0])
    link3 = Link([ang[3], 0, 10, 0])
    link4 = Link([ang[4], 0, 10, 0])
    end_link = Link([ang[5], 0, 0, 0])

    end = origin*base_link*link1*link2*link3*link4*end_link

    return end

joint = [
    pi/4,
    0,
    0,
    0,
    pi/4,
    pi/4
]

end = forward(joint)

xyz = end.position()
orientation = end.orientation()

print(xyz)