#!/usr/bin/env python2

import rospy
import tf
import tf.transformations
from tf2_ros import TransformException

from operator import add

rospy.init_node('gnss_broadcaster')
# Get params
gps_origin_frame = rospy.get_param('~gps_origin_frame', 'gps_origin')
gps_position_frame = rospy.get_param('~gps_position_frame', 'gps_position')
gps_orientation_frame = rospy.get_param('~gps_orientation_frame', 'gps_orientation')
gps_antenna_frame = rospy.get_param('~gps_antenna_frame', 'gps_antenna')
gps_base_frame = rospy.get_param('~gps_base_frame', 'gps_base_link')
gps_position_to_base = rospy.get_param('~gps_position_to_base', [[0, 0, 0], [0, 0, 0, 1]])
gps_orientation_to_base = rospy.get_param('~gps_orientation_to_base', [[0, 0, 0], [0, 0, 0, 1]])

tf_l = tf.TransformListener()

while not rospy.is_shutdown():
    rospy.sleep(0.1)

    t = rospy.Time(0)

    # Position
    try:
        tf_l.waitForTransform(gps_origin_frame, gps_position_frame, t, rospy.Duration(5))
    except TransformException:
        continue
    tf_p = tf_l.lookupTransform(gps_origin_frame, gps_position_frame, t)

    # Orientation
    try:
        tf_l.waitForTransform(gps_origin_frame, gps_orientation_frame, t, rospy.Duration(5))
    except TransformException:
        continue
    tf_o = tf_l.lookupTransform(gps_origin_frame, gps_orientation_frame, t)

    # Broadcast gps_antenna (frame representing the pose of the GPS antenna) and gps_base_link
    tf_b = tf.TransformBroadcaster()
    t = rospy.Time.now()
    # GPS antenna
    tf_b.sendTransform(tf_p[0], tf_o[1], t, gps_antenna_frame, gps_origin_frame)
    # Vehicle pose
    # TODO: Use static transform broadcaster from tf2_ros
    tf_b.sendTransform(gps_position_to_base[0], gps_orientation_to_base[1], t, gps_base_frame, gps_antenna_frame)
