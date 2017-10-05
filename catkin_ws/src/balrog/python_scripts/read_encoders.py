#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

import socket
import numpy as np
import struct

if __name__ == "__main__":
    # create a publisher that publishes messages of type Float64MultiArray on
    # the topic /encoder_data:
    pub = rospy.Publisher("/encoder_data", Float64MultiArray, queue_size=10)

    # initialize this code as a ROS node named read_encoders_node:
    rospy.init_node("read_encoders_node", anonymous=True)

    # create a TCP/IP socket:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # connect to the RPI server:
    host = "192.168.137.3" # (RPI IP address)
    port = 2100
    client_socket.connect((host, port))

    while not rospy.is_shutdown():
        data = client_socket.recv(4)
        no_of_bytes_to_read = struct.unpack("<L", data)[0]
        print "number of bytes to read: %d" % no_of_bytes_to_read

        data = client_socket.recv(8)
        #print data
        #print "***"
        message_type = struct.unpack("<d", data)[0]
        print "message type: %f" % message_type
        #print "^^^^^^"

        no_of_read_bytes = 0
        if no_of_bytes_to_read - 8 > 0:
            while no_of_read_bytes < no_of_bytes_to_read - 8:
                data = client_socket.recv(no_of_bytes_to_read - 8 - no_of_read_bytes)
                print len(data)
                no_of_read_bytes += len(data)
                #print data
                print "&&&&&&&&&&&&&&&"

        if message_type == 1:
            if len(data) == 136:
                print "THE SENSOR DATA HAS ARRIVED!!"
                lidar = data[0:8]
                lidar = struct.unpack("<d", lidar)[0]
                #print "lidar: %f" % lidar

                us_1 = data[8:16]
                us_1 = struct.unpack("<d", us_1)[0]
                #print "us 1: %f" % us_1

                us_2 = data[16:24]
                us_2 = struct.unpack("<d", us_2)[0]
                #print "us 2: %f" % us_2

                us_3 = data[24:32]
                us_3 = struct.unpack("<d", us_3)[0]
                #print "us 3: %f" % us_3

                us_4 = data[32:40]
                us_4 = struct.unpack("<d", us_4)[0]
                #print "us 4: %f" % us_4

                gyro_1 = data[40:48]
                gyro_1 = struct.unpack("<d", gyro_1)[0]
                #print "gyro 1: %f" % gyro_1

                gyro_2 = data[48:56]
                gyro_2 = struct.unpack("<d", gyro_2)[0]
                #print "gyro 2: %f" % gyro_2

                gyro_3 = data[56:64]
                gyro_3 = struct.unpack("<d", gyro_3)[0]
                #print "gyro 3: %f" % gyro_3

                acc_1 = data[64:72]
                acc_1 = struct.unpack("<d", acc_1)[0]
                #print "acc 1: %f" % acc_1

                acc_2 = data[72:80]
                acc_2 = struct.unpack("<d", acc_2)[0]
                #print "acc 2: %f" % acc_2

                acc_3 = data[80:88]
                acc_3 = struct.unpack("<d", acc_3)[0]
                #print "acc 3: %f" % acc_3

                magn_1 = data[88:96]
                magn_1 = struct.unpack("<d", magn_1)[0]
                #print "magn 1: %f" % magn_1

                magn_2 = data[96:104]
                magn_2 = struct.unpack("<d", magn_2)[0]
                #print "magn 2: %f" % magn_2

                magn_3 = data[104:112]
                magn_3 = struct.unpack("<d", magn_3)[0]
                #print "magn 3: %f" % magn_3

                odoRight = data[112:120]
                odoRight = struct.unpack("<d", odoRight)[0]
                print "odoRight: %f" % odoRight

                odoLeft = data[120:128]
                odoLeft = struct.unpack("<d", odoLeft)[0]
                print "odoLeft: %f" % odoLeft

                time = data[128:136]
                time = struct.unpack("<d", time)[0]
                print "time: %f" % time


                msg = Float64MultiArray()
                data = [odoRight, odoLeft]
                msg.data = data
                pub.publish(msg)
