 #!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import rospy
import serial
import threading
import time

import pynmea2
from std_msgs.msg import Header
from gps_common.msg import GPSFix, GPSStatus

class RTK:
    def __init__(self, port='/dev/ttyUSB0', ros_pub=None):
        self.serial = serial.Serial(port, 115200)
        self.ros_pub = ros_pub
        if self.ros_pub is None:
            print('No ros_pub !')

        # self.speed = 0.
        self.yaw = 0.
        self.err_yaw = 0.
        self.latitude = 0.
        self.longitude = 0.
        # time
        self.last_gps = 0
        self.last_heading = 0
        # flag
        self.new_gps = False
        self.new_heading = False

        recv_thread = threading.Thread(
            target = self.read_serial, args=()
        )
        recv_thread.start()

    def read_serial(self):
        while True:
            try:
                line = self.serial.readline()
                line = line.decode()
                self.parse(line)
            except:
                pass
        
            try:
                # get both data, publish
                if self.new_gps and self.new_heading:
                    # clear flag
                    self.new_gps = False
                    self.new_heading = False

                    # check timeout
                    timeout_valid = self.check_signal(timeout = 0.1)

                    ros_msg = GPSFix()
                    ros_msg.header = Header()
                    ros_msg.header.stamp = rospy.Time.now()
                    ros_msg.header.frame_id = 'gnss_link'
                    ros_msg.latitude = self.latitude
                    ros_msg.longitude = self.longitude
                    ros_msg.dip = self.yaw
                    ros_msg.err_dip = self.err_yaw
                    ros_msg.status = GPSStatus()
                    ros_msg.status.status = 2 if timeout_valid else 0
                    # ros_msg.speed = self.speed
                    if self.ros_pub is not None:
                        self.ros_pub.publish(ros_msg)
            except:
                pass
        
    def check_signal(self, timeout = 0.1):
        now = time.time()
        if (now-self.last_gps) < timeout and (now-self.last_heading) < timeout:
            return True
        else:
            return False

    def parse(self, data):
        if data[:9] == '#HEADINGA':
            sp_line = data.split(',')
            self.yaw = float(sp_line[12])
            self.err_yaw = float(sp_line[15])
            if abs(self.err_yaw) > 0.3:
                self.err_yaw = 0.1
                print('get error err_yaw', float(sp_line[15]))

            self.last_heading = time.time()
            self.new_heading = True

        if data[:6] == '$GPGGA':
            rmc = pynmea2.parse(data)
            self.latitude = rmc.latitude
            self.longitude = rmc.longitude

            self.last_gps = time.time()
            self.new_gps = True

def open_serial():
    serial_list = [item for item in os.listdir('/dev/') if item[:6]=='ttyUSB']
    if len(serial_list) > 0:
        for item in serial_list:
            os.system('sudo chmod 777 /dev/' + item)

    if len(serial_list) > 1:
        print('More than one USB found:')
        for item in serial_list:
            print(item)
        print('Using', serial_list[0])

    return serial_list[0]


if __name__ == '__main__':
    rospy.init_node('rtk', anonymous=True)
    rtk_pub = rospy.Publisher('/jzhw/gps/fix', GPSFix, queue_size=0)

    usb = open_serial()
    rtk = RTK('/dev/' + usb, rtk_pub)

    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
        rate.sleep()
