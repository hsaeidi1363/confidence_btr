#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client
from std_msgs.msg import Bool


def callback(config):
    rospy.loginfo("Config set to {emitter_enabled}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dyn_laser")
    laser_pub = rospy.Publisher('d415_laser_off',Bool, queue_size = 1)

    client = dynamic_reconfigure.client.Client("camera/stereo_module", timeout=3, config_callback=callback)

    r = rospy.Rate(2)
    x = 0
    laser_off = False
    while not rospy.is_shutdown():
        x = 1-x
        if x == 1:
            laser_off = False
        elif x == 0:
            laser_off = True
        client.update_configuration({"emitter_enabled":x})
        laser_pub.publish(laser_off)
        r.sleep()
