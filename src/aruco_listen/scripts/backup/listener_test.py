#!/usr/bin/env python3

import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('usb_cam', 'marker_id1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #angular = 4 * math.atan2(trans[1], trans[0])
        #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        #cmd = geometry_msgs.msg.Twist()
        #cmd.linear.x = linear
        #cmd.angular.z = angular
        print(rot)
        rate.sleep()

