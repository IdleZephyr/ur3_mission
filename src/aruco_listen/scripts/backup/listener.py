#!/usr/bin/env python3

import roslib
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('usb_cam', 'marker_id1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        x1, y2, z3 = trans[0], trans[1], trans[2]
        print(y)
        rate.sleep()

