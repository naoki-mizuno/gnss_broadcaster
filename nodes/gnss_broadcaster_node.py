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

# Populated when ready
latest_fix = None
latest_heading = None


class Fix:
    def __init__(self, msg):
        self.header = msg.header
        rest, self.checksum = msg.sentence.split('*')
        self.msg_id, self.utc, self.lat, self.lat_dir, \
        self.lon, self.lon_dir, self.quality, self.num_sat, \
        self.hdop, self.orthometric_height, self.orthometric_height_unit, self. geoid_sep, \
        self.geoid_sep_unit, self.age, self.ref_station_id = rest.split(',')

        self.lat = float(self.lat)
        self.lon = float(self.lon)
        self.orthometric_height = float(self.orthometric_height)
        self.geoid_sep = float(self.geoid_sep)
        self.height = self.orthometric_height + self.geoid_sep


def heading_callback(msg):
    global latest_heading
    latest_heading = msg
    send_transform()


def nmea_callback(msg):
    global required_quality
    global latest_fix
    latest_fix = Fix(msg)

    if latest_fix.quality not in required_quality:
        latest_fix = None
        return

    send_transform()


def send_transform():
    global gps_origin_frame
    global gps_antenna_frame
    global plane
    global latest_fix
    global latest_heading

    if latest_fix is None or latest_heading is None:
        return

    converter = gnss_broadcaster.GeoPosConv()
    converter.set_plane(plane)
    converter.set_llh_nmea_degrees(latest_fix.lat, latest_fix.lon, latest_fix.height)
    x = converter.y()
    y = converter.x()
    z = converter.z()
    translation = (x, y, z)
    q = latest_heading.quaternion
    rotation = (q.x, q.y, q.z, q.w)
    tf_b = tf.TransformBroadcaster()
    tf_b.sendTransform(translation,
                       rotation,
                       latest_fix.header.stamp,
                       gps_antenna_frame,
                       gps_origin_frame)

rospy.Subscriber('nmea_sentence', Sentence, nmea_callback, queue_size=1)
rospy.Subscriber('heading', QuaternionStamped, heading_callback, queue_size=1)

rospy.loginfo('Using plane {0}'.format(plane))

rospy.spin()
