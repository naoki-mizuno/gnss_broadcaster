#!/usr/bin/env python2

import rospy
import tf.transformations
from nmea_msgs.msg import Sentence
from geometry_msgs.msg import Quaternion, QuaternionStamped
import math

from osgeo import osr

rospy.init_node('gnss_broadcaster')
# Get params
gps_origin_frame = rospy.get_param('~gps_origin_frame', 'gps_origin')
gps_antenna_frame = rospy.get_param('~gps_antenna_frame', 'gps_antenna')
plane = rospy.get_param('~plane')
if plane < 1 or plane > 19:
    raise('Invalid plane number: {0}'.format(plane))
# Rotation of the GNSS compass
gps_compass_offset = rospy.get_param('~gps_compass_offset', 0)
required_quality = rospy.get_param('~required_quality', ['4', '5'])

# Populated when ready
latest_fix = None
latest_orientation = None

wgs84 = osr.SpatialReference()
wgs84.ImportFromEPSG(4326)
japan_plane_cs = osr.SpatialReference()
# Coordinate number starts from 1
japan_plane_cs.ImportFromEPSG(2443 + plane - 1)
coord_tf = osr.CoordinateTransformation(wgs84, japan_plane_cs)


class Fix:
    def __init__(self, msg):
        self.header = msg.header
        rest, self.checksum = msg.sentence.split('*')
        self.msg_id, self.utc, self.lat, self.lat_dir, \
        self.lon, self.lon_dir, self.quality, self.num_sat, \
        self.hdop, self.orthometric_height, self.orthometric_height_unit, self. geoid_sep, \
        self.geoid_sep_unit, self.age, self.ref_station_id = rest.split(',')

        # 3820.9247885 -> 38 deg 20.924788' -> 38 + 20.924788 / 60 -> 38.348746475
        self.lat = float(self.lat)
        self.lat = math.floor(self.lat / 100) + (self.lat % 100) / 60
        self.lon = float(self.lon)
        self.lon = math.floor(self.lon / 100) + (self.lon % 100) / 60
        self.orthometric_height = float(self.orthometric_height)
        self.geoid_sep = float(self.geoid_sep)
        self.height = self.orthometric_height + self.geoid_sep


def heading_callback(msg):
    global latest_orientation
    global gps_compass_offset

    q = (msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
    hdg = tf.transformations.euler_from_quaternion(q)[2]
    yaw = math.pi / 2 - gps_compass_offset - hdg

    latest_orientation = QuaternionStamped()
    latest_orientation.header = msg.header
    latest_orientation.quaternion = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, yaw))
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
    global latest_orientation
    global coord_tf

    if latest_fix is None or latest_orientation is None:
        return

    x, y, _ = coord_tf.TransformPoint(latest_fix.lon, latest_fix.lat)
    z = latest_fix.height
    translation = (x, y, z)
    q = latest_orientation.quaternion
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
