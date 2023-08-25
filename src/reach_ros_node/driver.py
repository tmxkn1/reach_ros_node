# Software License Agreement (BSD License)
#
# Copyright (c) 2018, Patrick Geneva
# Copyright (c) 2013, Eric Perko
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the names of the authors nor the names of their
#    affiliated organizations may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import math

import rospy

from sensor_msgs.msg import NavSatFix, NavSatStatus, TimeReference
from geometry_msgs.msg import TwistStamped

from reach_ros_node.checksum_utils import check_nmea_checksum
import reach_ros_node.parser


class RosNMEADriver(object):
    def __init__(self):
        self.fix_pub = rospy.Publisher('tcpfix', NavSatFix, queue_size=1)
        self.vel_pub = rospy.Publisher('tcpvel', TwistStamped, queue_size=1)
        self.timeref_pub = rospy.Publisher('tcptime', TimeReference, queue_size=1)
        self.frame_timeref = rospy.get_param('~frame_timeref', 'gps')
        self.frame_gps = rospy.get_param('~frame_gps', 'gps')
        self.use_rostime = rospy.get_param('~use_rostime', True)
        self.use_rmc = rospy.get_param('~use_rmc', False)
        self.msg_fix = NavSatFix()
        self.msg_vel = TwistStamped()
        self.msg_timeref = TimeReference()
        # state flags
        self.has_fix = False
        self.has_std = False
        self.has_vel = False
        self.has_timeref = False

    def process_line(self, nmea_string):
        if not check_nmea_checksum(nmea_string):
            rospy.logwarn("Received a sentence with an invalid checksum. Sentence was: %s" % repr(nmea_string))
            return
        # Parse the NMEA sentence
        parsed_sentence = reach_ros_node.parser.parse_nmea_sentence(nmea_string)
        if not parsed_sentence:
            rospy.logwarn("Failed to parse NMEA sentence. Sentence was: %s" % nmea_string)
            return
        # Extract and publish data based on sentence type
        if self.use_rmc:
            if 'RMC' in parsed_sentence:
                self.parse_RMC(parsed_sentence['RMC'])
                self.parse_time(parsed_sentence['RMC'])
            else:
                pass
        elif'GGA' in parsed_sentence:
            self.parse_GGA(parsed_sentence['GGA'])
            self.parse_time(parsed_sentence['GGA'])
        elif 'GST' in parsed_sentence:
            self.parse_GST(parsed_sentence['GST'])
        elif 'VTG' in parsed_sentence:
            self.parse_VTG(parsed_sentence['VTG'])
        elif 'RMC' in parsed_sentence:
            self.parse_RMC(parsed_sentence['RMC'])
        # Publish fix messages
        if self.has_fix and self.has_std:
            self.fix_pub.publish(self.msg_fix)
            self.msg_fix = NavSatFix()
            self.has_fix = False
            self.has_std = False
        if self.has_vel:
            self.vel_pub.publish(self.msg_vel)
            self.msg_vel = TwistStamped()
            self.has_vel = False
        if self.has_timeref:
            self.timeref_pub.publish(self.msg_timeref)
            self.msg_timeref = TimeReference()
            self.has_timeref = False

    def parse_GGA(self, data):
        self.msg_fix.header.stamp = self.get_timestamp(data)
        self.msg_fix.header.frame_id = self.frame_gps
        # Update the fix message
        gps_qual = data['fix_type']
        if gps_qual == 0:
            self.msg_fix.status.status = NavSatStatus.STATUS_NO_FIX
        elif gps_qual == 1:
            self.msg_fix.status.status = NavSatStatus.STATUS_FIX
        elif gps_qual == 2:
            self.msg_fix.status.status = NavSatStatus.STATUS_SBAS_FIX
        elif gps_qual in (4, 5):
            self.msg_fix.status.status = NavSatStatus.STATUS_GBAS_FIX
        else:
            self.msg_fix.status.status = NavSatStatus.STATUS_NO_FIX
        self.msg_fix.status.service = NavSatStatus.SERVICE_GPS
        # Set lat lon position
        self.msg_fix.latitude, self.msg_fix.longitude = self.get_lat_lon(data)
        # Altitude is above ellipsoid, so adjust for mean-sea-level
        self.msg_fix.altitude = data['altitude'] + data['mean_sea_level']
        self.has_fix = True

    def parse_GST(self, data):
        self.msg_fix.position_covariance[0] = pow(data['latitude_sigma'], 2)
        self.msg_fix.position_covariance[4] = pow(data['longitude_sigma'], 2)
        self.msg_fix.position_covariance[8] = pow(data['altitude_sigma'], 2)
        self.msg_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        self.has_std = True

    def parse_VTG(self, data):
        self.msg_vel.header.stamp = self.get_timestamp(data)
        self.msg_vel.header.frame_id = self.frame_gps
        # Calculate the change in orientatoin
        self.msg_vel.twist.linear.x = data['speed'] * math.sin(data['ori_true'])
        self.msg_vel.twist.linear.y = data['speed'] * math.cos(data['ori_true'])
        self.has_vel = True

    def parse_RMC(self, data):
        self.msg_fix.header.stamp = self.get_timestamp(data)
        self.msg_fix.header.frame_id = self.frame_gps
        # Update the fix message
        if data['fix_valid']:
            self.msg_fix.status.status = NavSatStatus.STATUS_FIX
        else:
            self.msg_fix.status.status = NavSatStatus.STATUS_NO_FIX
        self.msg_fix.status.service = NavSatStatus.SERVICE_GPS
        # Set lat lon position
        self.msg_fix.latitude, self.msg_fix.longitude = self.get_lat_lon(data)
        # RMC does not provide the height and covariance
        self.msg_fix.altitude = float('NaN')
        self.msg_fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.has_fix = True
        self.has_std = True
        # Set velocity message
        self.msg_vel.header.stamp = self.get_timestamp(data)
        self.msg_vel.header.frame_id = self.frame_gps
        self.msg_vel.twist.linear.x = data['speed'] * math.sin(data['true_course'])
        self.msg_vel.twist.linear.y = data['speed'] * math.cos(data['true_course'])
        self.has_vel = True

    def parse_time(self, data):
        if math.isnan(data['utc_time']):
            return
        self.msg_timeref.header.stamp = self.get_timestamp(data)
        self.msg_timeref.header.frame_id = self.frame_timeref
        # Set GPS time reference
        self.msg_timeref.time_ref = rospy.Time.from_sec(data['utc_time'])
        self.msg_timeref.source = self.frame_timeref
        self.has_timeref = True

    def get_timestamp(self, data):
        if self.use_rostime:
            return rospy.get_rostime()
        else:
            time = data['utc_time'] if 'utc_time' in data.keys() else 0
            return rospy.Time.from_sec(time)

    def get_lat_long(self, data):
        latitude = data['latitude']
        if data['latitude_direction'] == 'S':
            latitude = -latitude
        self.msg_fix.latitude = latitude
        longitude = data['longitude']
        if data['longitude_direction'] == 'W':
            longitude = -longitude
        return latitude, longitude