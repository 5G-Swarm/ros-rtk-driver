 #!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import serial
import threading
import time

import pynmea2
from std_msgs.msg import Header
from gps_common.msg import GPSFix

class RTK:
    def __init__(self, port='/dev/ttyUSB0', ros_pub=None):
        self.serial = serial.Serial(port, 115200)
        self.ros_pub = ros_pub
        if self.ros_pub is None:
            print('No ros_pub !')

        self.speed = 0.
        self.yaw = 0.
        self.latitude = 0.
        self.longitude = 0.

        recv_thread = threading.Thread(
            target = self.read_serial, args=()
        )
        recv_thread.start()

    def read_serial(self):
        while True:
            line = self.serial.readline()
            # line = '$GPRMC,144326.00,A,5107.0017737,N,11402.3291611,W,0.080,323.3,210307,0.0,E,A*20'
            self.parse(line)

            ros_msg = GPSFix()
            ros_msg.header = Header()
            ros_msg.header.stamp = rospy.Time.now()
            ros_msg.latitude = self.latitude
            ros_msg.longitude = self.longitude
            ros_msg.track = self.yaw
            ros_msg.speed = self.speed
            if self.ros_pub is not None:
                self.ros_pub.publish(ros_msg)
        

    def parse(self, data):
        if data[:6] == '$GPRMC':
            rmc = pynmea2.parse(data)
            self.latitude = rmc.latitude
            self.longitude = rmc.longitude
            self.speed = rmc.spd_over_grnd
            self.yaw = rmc.true_course


if __name__ == '__main__':
    rospy.init_node('rtk', anonymous=True)
    rtk_pub = rospy.Publisher('/rtk', GPSFix, queue_size=0)

    rtk = RTK('/dev/ttyUSB0', rtk_pub)

    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
