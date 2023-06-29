#!/bin/env python
import rospy
import numpy as np
from triad_openvr import triad_openvr

import geometry_msgs.msg
import sensor_msgs.msg

from ur_control import conversions, transformations


class ViveROS():
    def __init__(self) -> None:
        rospy.init_node("vive_ROS")

        config_file = rospy.get_param("/vive/config_file", None)
        
        self.vr = triad_openvr(configfile_path=config_file)

        self.topic_map = {}
        self.haptic_feedback_sub = rospy.Subscriber("/vive/feedback", sensor_msgs.msg.JoyFeedback, self.haptic_feedback)

        publishing_rate = int(rospy.get_param("/vive/publishing_rate", 100))
        self.pub_rate = rospy.Rate(publishing_rate)

        self.vr.print_discovered_objects()

    def run(self):

        update_devices_interval = 5  # seconds
        update_devices_start_time = rospy.get_time()

        while not rospy.is_shutdown():
            # Check for new controllers, remove disconnected ones every 30 seconds

            if rospy.get_time() - update_devices_start_time > update_devices_interval:
                update_devices_start_time = rospy.get_time()
                self.vr.poll_vr_events()

            detected_controllers = self.vr.object_names["Controller"]

            for device_name in detected_controllers:

                self.publish_twist(device_name)
                self.publish_controller_input(device_name)

            # publishing rate
            self.pub_rate.sleep()

    def publish_controller_input(self, device_name):
        button_state_topic = self.topic_map.get(device_name, rospy.Publisher("/vive/" + device_name + "/joy", sensor_msgs.msg.Joy, queue_size=10))

        controller_inputs = self.vr.devices[device_name].get_controller_inputs()

        inputs_msg = sensor_msgs.msg.Joy()
        inputs_msg.header.frame_id = device_name
        
        inputs_msg.buttons = [
            int(controller_inputs['menu_button']),
            int(controller_inputs['trackpad_pressed']),
            int(controller_inputs['trigger']),
            int(controller_inputs['grip_button']),
        ]

        button_state_topic.publish(inputs_msg)

    def publish_twist(self, device_name):
        # Controller velocity
        # x and z - diagonals, y - up/down
        controller_twist = self.vr.devices[device_name].get_velocity()
        
        if controller_twist is None:
            return

        # Rotate twist to align with ROS world (x forward/backward, y right/left, z up/down)
        rotation = transformations.quaternion_from_euler(0.0, np.deg2rad(45), np.deg2rad(-90))
        rotation = transformations.rotate_quaternion_by_rpy(0.0, np.deg2rad(-90), 0.0, rotation)
        controller_twist[:3] = transformations.quaternion_rotate_vector(rotation, controller_twist[:3])
        twist_topic = self.topic_map.get(device_name, rospy.Publisher("/vive/" + device_name + "/twist", geometry_msgs.msg.Twist, queue_size=10))

        twist_msg = geometry_msgs.msg.Twist()
        twist_msg.linear = conversions.to_vector3(controller_twist[:3])
        twist_msg.angular = conversions.to_vector3(controller_twist[3:])

        twist_topic.publish(twist_msg)

    def haptic_feedback(self, msg: sensor_msgs.msg.JoyFeedback):
        device = self.vr.devices.get(msg.id, None)
        if device:
            device.trigger_haptic_pulse(duration_micros=msg.intensity)


if __name__ == '__main__':
    vive_ros = ViveROS()
    vive_ros.run()
