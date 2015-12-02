#!/usr/bin/env python

PACKAGE = "youbot_ros_simple_trajectory"
import roslib;roslib.load_manifest(PACKAGE)
import rospy

import dynamic_reconfigure.client

def callback(config):
    rospy.loginfo("Config set to {arm1}, {arm2}, {arm3}, {arm4}, {arm5}, {gripper_l_param}, {gripper_r_param}".format(**config))

if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("youbot_dy_server", timeout=30, config_callback=callback)

    r = rospy.Rate(0.1)
    #x = 0
    #b = False 
    while not rospy.is_shutdown():
        #x = x+1
        #if x>10:
        #    x=0
        #b = not b
       # client.update_configuration({"int_param":x, "double_param":(1/(x+1)), "str_param":str(rospy.get_rostime()), "bool_param":b, "size":1})
    	r.sleep()
