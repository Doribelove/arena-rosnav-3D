#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time

import rospy
import serial
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def parse_gga(line):
    if not (line.startswith("$GNGGA") or line.startswith("$GPGGA")):
        return None

    parts = line.split(",")
    if len(parts) < 10:
        return None
    if parts[2] == "" or parts[4] == "" or parts[6] == "0":
        return None

    lat_raw = float(parts[2])
    lat = int(lat_raw / 100) + (lat_raw % 100) / 60.0
    if parts[3] == "S":
        lat = -lat

    lon_raw = float(parts[4])
    lon = int(lon_raw / 100) + (lon_raw % 100) / 60.0
    if parts[5] == "W":
        lon = -lon

    fix_quality = int(parts[6]) if parts[6].isdigit() else 0
    satellites = int(parts[7]) if parts[7].isdigit() else 0
    altitude = float(parts[9]) if parts[9] else 0.0
    return lat, lon, altitude, fix_quality, satellites


def gps_to_xy(lat, lon, origin_lat, origin_lon):
    radius = 6378137.0
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    ref_lat = math.radians(origin_lat)
    x = radius * d_lon * math.cos(ref_lat)
    y = radius * d_lat
    return x, y


def yaw_from_quaternion(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class GpsLocalizationNode:
    def __init__(self):
        self.port = rospy.get_param("~port", "/dev/ttyUSB1")
        self.baud_rate = int(rospy.get_param("~baud_rate", 115200))
        self.map_frame = rospy.get_param("~map_frame", "camera_init")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.pose_topic = rospy.get_param("~pose_topic", "/gps/pose")
        self.odom_topic = rospy.get_param("~odom_topic", "/gps/odom")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.origin_lat = rospy.get_param("~origin_lat", None)
        self.origin_lon = rospy.get_param("~origin_lon", None)
        if self.origin_lat == "":
            self.origin_lat = None
        if self.origin_lon == "":
            self.origin_lon = None
        if self.origin_lat is not None:
            self.origin_lat = float(self.origin_lat)
        if self.origin_lon is not None:
            self.origin_lon = float(self.origin_lon)
        self.publish_rate = float(rospy.get_param("~publish_rate", 10.0))
        self.serial_timeout = float(rospy.get_param("~serial_timeout", 0.2))
        self.broadcast_tf = bool(rospy.get_param("~broadcast_tf", True))

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.fix_pub = rospy.Publisher(self.fix_topic, NavSatFix, queue_size=10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.ser = serial.Serial(self.port, self.baud_rate, timeout=self.serial_timeout)
        rospy.loginfo("GPS localization reading %s at %d baud", self.port, self.baud_rate)

    def publish_fix(self, stamp, lat, lon, altitude, fix_quality):
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = "gps"
        msg.status.status = NavSatStatus.STATUS_FIX if fix_quality > 0 else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = altitude
        self.fix_pub.publish(msg)

    def lookup_odom_base(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.05),
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                2.0,
                "GPS localization cannot lookup %s -> %s yet: %s",
                self.odom_frame,
                self.base_frame,
                exc,
            )
            return None

    def publish_pose_and_tf(self, stamp, x, y, altitude):
        odom_base = self.lookup_odom_base()
        odom_x = odom_y = odom_yaw = 0.0
        if odom_base is not None:
            odom_x = odom_base.transform.translation.x
            odom_y = odom_base.transform.translation.y
            odom_yaw = yaw_from_quaternion(odom_base.transform.rotation)

        q = quaternion_from_euler(0.0, 0.0, odom_yaw)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.map_frame
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = altitude
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        self.pose_pub.publish(pose)

        gps_odom = Odometry()
        gps_odom.header = pose.header
        gps_odom.child_frame_id = self.base_frame
        gps_odom.pose.pose = pose.pose
        self.odom_pub.publish(gps_odom)

        if not self.broadcast_tf:
            return

        cos_yaw = math.cos(odom_yaw)
        sin_yaw = math.sin(odom_yaw)
        map_odom_x = x - (cos_yaw * odom_x - sin_yaw * odom_y)
        map_odom_y = y - (sin_yaw * odom_x + cos_yaw * odom_y)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = map_odom_x
        tf_msg.transform.translation.y = map_odom_y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(tf_msg)

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            raw = self.ser.readline().decode("utf-8", errors="ignore").strip()
            parsed = parse_gga(raw)
            if parsed is None:
                rate.sleep()
                continue

            lat, lon, altitude, fix_quality, satellites = parsed
            if self.origin_lat is None or self.origin_lon is None:
                self.origin_lat = lat
                self.origin_lon = lon
                rospy.loginfo(
                    "GPS origin set to lat=%.8f lon=%.8f; satellites=%d",
                    self.origin_lat,
                    self.origin_lon,
                    satellites,
                )

            x, y = gps_to_xy(lat, lon, self.origin_lat, self.origin_lon)
            stamp = rospy.Time.now()
            self.publish_fix(stamp, lat, lon, altitude, fix_quality)
            self.publish_pose_and_tf(stamp, x, y, altitude)
            rate.sleep()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    rospy.init_node("gps_localization_node")
    node = None
    try:
        node = GpsLocalizationNode()
        node.spin()
    finally:
        if node is not None:
            node.close()
