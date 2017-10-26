#!/usr/bin/env python2

import rospy
import tf
import tf.transformations

from operator import add

rospy.init_node('gnss_broadcaster')
# Get params
gps_origin_frame = rospy.get_param('~gps_origin_frame', 'gps_origin')
gps_position_frame = rospy.get_param('~gps_position_frame', 'gps_position')
gps_position_to_base = rospy.get_param('~gps_position_to_base', [[0, 0, 0], [0, 0, 0, 1]])
gps_orientation_frame = rospy.get_param('~gps_orientation_frame', 'gps_orientation')
gps_orientation_to_base = rospy.get_param('~gps_orientation_to_base', [[0, 0, 0], [0, 0, 0, 1]])
gps_base_frame = rospy.get_param('~gps_base_frame', 'gps_base_link')

tf_l = tf.TransformListener()

while not rospy.is_shutdown():
    rospy.sleep(0.1)

    frames = tf_l.getFrameStrings()
    t = rospy.Time(0)

    # Position
    if gps_position_frame not in frames:
        continue
    tf_l.waitForTransform(gps_origin_frame, gps_position_frame, t, rospy.Duration(5))
    tf_p = tf_l.lookupTransform(gps_origin_frame, gps_position_frame, t)

    # Orientation
    if gps_orientation_frame not in frames:
        continue
    tf_l.waitForTransform(gps_origin_frame, gps_orientation_frame, t, rospy.Duration(5))
    tf_o = tf_l.lookupTransform(gps_origin_frame, gps_orientation_frame, t)

    # Add the offset of the antenna
    gps_base = (
        list(map(add, tf_p[0], gps_position_to_base[0])),
        tf.transformations.quaternion_multiply(tf_o[1], gps_orientation_to_base[1])
    )

    # Broadcast gps_base_frame
    tf_b = tf.TransformBroadcaster()
    tf_b.sendTransform(gps_base[0], gps_base[1], rospy.Time.now(), gps_base_frame, gps_origin_frame)
