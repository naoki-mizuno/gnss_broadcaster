#!/usr/bin/env python2

import rospy
import tf

rospy.init_node('gnss_broadcaster')
# Get params
gps_origin_frame = rospy.get_param('~gps_origin_frame', 'gps_origin')
gps_position_frame = rospy.get_param('~gps_position_frame', 'gps_position')
gps_position_to_base = rospy.get_param('~gps_position_to_base', [[0, 0, 0], [0, 0, 0, 1]])
gps_orientation_frame = rospy.get_param('~gps_orientation_frame', 'gps_orientation')
gps_orientation_to_base = rospy.get_param('~gps_orientation_to_base', [[0, 0, 0], [0, 0, 0, 1]])
gps_base_frame = rospy.get_param('~gps_base_frame', 'gps_base_link')

while not rospy.is_shutdown():
    t = rospy.Time.now()

    # Position
    tf_l_p = tf.TransformListener()
    tf_l_p.waitForTransform(gps_position_frame, gps_origin_frame, t, rospy.Duration(5))
    tf_p = tf_l_p.lookupTransform(gps_position_frame, gps_origin_frame, t)

    # Orientation
    tf_l_o = tf.TransformListener()
    tf_l_o.waitForTransform(gps_orientation_frame, gps_origin_frame, t, rospy.Duration(5))
    tf_o = tf_l_o.lookupTransform(gps_orientation_frame, gps_origin_frame, t)

    # (Virtual) antenna's pose before shifting it over to gps_base_link
    gps_base = (tf_p[0], tf_o[1])
    # Add the offset of the antenna
    gps_base[0] += gps_position_to_base[0]
    gps_base[1] += gps_orientation_to_base[1]

    # Broadcast gps_base_frame
    tf_b = tf.TransformBroadcaster()
    tf_b.sendTransform(gps_base[0], gps_base[1], rospy.Time.now(), gps_base_frame, gps_origin_frame)
