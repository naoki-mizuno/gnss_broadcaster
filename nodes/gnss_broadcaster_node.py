#!/usr/bin/env python2

import rospy
import tf2_ros
import tf.transformations
from nmea_msgs.msg import Sentence
from geometry_msgs.msg import QuaternionStamped

import gnss_broadcaster

rospy.init_node('gnss_broadcaster')
# Get params
gps_origin_frame = rospy.get_param('~gps_origin_frame', 'gps_origin')
gps_antenna_frame = rospy.get_param('~gps_antenna_frame', 'gps_antenna')
plane = rospy.get_param('~plane')
required_quality = rospy.get_param('~required_quality', ['4', '5'])

latest_heading = None


def heading_callback(msg):
    global latest_heading

    latest_heading = msg


def nmea_callback(msg):
    global gps_origin_frame
    global gps_antenna_frame
    global plane
    global required_quality
    global latest_heading

    rest, checksum = msg.sentence.split('*')
    msg_id, utc, lat, lat_dir, \
    lon, lon_dir, quality, num_sat, \
    hdop, height, _, geoid_sep, \
    _, age, ref_station_id = rest.split(',')

    if quality not in required_quality:
        return

    converter = gnss_broadcaster.GeoPosConv()
    converter.set_plane(plane)

    converter.set_llh_nmea_degrees(lat, lon, height)
    x = converter.y()
    y = converter.x()
    z = converter.z()
    translation = (x, y, z)
    rotation = tf.transformations.quaternion_from_euler(0, 0, 0)
    if not latest_heading:
        rotation = latest_heading.quaternion
    tf_b = tf.TransformBroadcaster()
    tf_b.sendTransform(translation,
                       rotation,
                       msg.header.stamp,
                       gps_antenna_frame,
                       gps_origin_frame)

rospy.Subscriber('nmea_sentence', Sentence, nmea_callback, queue_size=1)
rospy.Subscriber('heading', QuaternionStamped, heading_callback, queue_size=1)

rospy.loginfo('Using plane {0}'.format(plane))

rospy.spin()
