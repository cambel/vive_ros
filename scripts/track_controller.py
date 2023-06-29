#!/bin/env python
import rospy
import geometry_msgs.msg
import numpy as np
from triad_openvr import triad_openvr
from ur_control import conversions, transformations

vr = triad_openvr()

rospy.init_node("none")
topic_map = {}
rate = rospy.Rate(100)

vr.print_discovered_objects()

while not rospy.is_shutdown():

    detected_controllers = vr.object_names["Controller"]

    for device in detected_controllers:
        serial = vr.devices[device].get_serial().replace("-", "_")

        # Controller velocity
        # x and z - diagonals, y - up/down
        controller_twist = vr.devices[device].get_velocities()
        
        # Rotate twist to align with ROS world (x forward/backward, y right/left, z up/down) 
        rotation = transformations.quaternion_from_euler(0.0, np.deg2rad(45), np.deg2rad(-90))
        rotation = transformations.rotate_quaternion_by_rpy(0.0, np.deg2rad(-90), 0.0, rotation)
        controller_twist[:3] = transformations.quaternion_rotate_vector(rotation, controller_twist[:3])

        twist_topic = topic_map.get(serial, rospy.Publisher("/vive/controller_" + serial + "/twist", geometry_msgs.msg.Twist, queue_size=10))

        twist_msg = geometry_msgs.msg.Twist()
        twist_msg.linear = conversions.to_vector3(controller_twist[:3])
        twist_msg.angular = conversions.to_vector3(controller_twist[3:])

        twist_topic.publish(twist_msg)

    # publishing rate
    rate.sleep()
