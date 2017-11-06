#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

import socket
import numpy as np
import struct

import threading

class Communicator:
    def __init__(self):
        # initialize this code as a ROS node named laptop_comm_node:
        rospy.init_node("laptop_comm_node", anonymous=True)

        # subscribe to the topic with control signals:
        rospy.Subscriber("/control_signals", Float64MultiArray, self.control_callback)

        # create a publisher that publishes messages of type Float64MultiArray
        # on the topic /encoder_data:
        self.encoder_pub = rospy.Publisher("/encoder_data", Float64MultiArray, queue_size=10)

        # create a TCP/IP socket:
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        self.socket_lock = threading.Lock()

        # connect to the RPI server:
        self.host = "192.168.137.3" # (RPI IP address)
        self.port = 2100
        self.client_socket.connect((self.host, self.port))

        omega_l = 0
        omega_r = 0
        data1 = np.double(omega_l)
        data2 = np.double(omega_r)
        data3 = np.double(10)
        data4 = np.double(20)
        data5 = np.double(30)
        checksum = np.double(data1 + data2 + data3 + data4 + data5)
        print "socket_lock.acquire()"
        self.socket_lock.acquire()
        print "socket_lock acquired!"
        self.client_socket.sendall(np.int32(48)) # (6 doubles = 6*8 bytes = 48 bytes)
        self.client_socket.sendall(data1)
        self.client_socket.sendall(data2)
        self.client_socket.sendall(data3)
        self.client_socket.sendall(data4)
        self.client_socket.sendall(data5)
        self.client_socket.sendall(checksum)
        self.socket_lock.release()

    # define the callback function for the /control_signals subscriber:
    def control_callback(self, msg_obj):
        print "control_callback"

        ctrl_signals = msg_obj.data

        omega_l = ctrl_signals[0]
        omega_r = ctrl_signals[1]

        data1 = np.double(omega_l)
        data2 = np.double(omega_r)
        data3 = np.double(10)
        data4 = np.double(20)
        data5 = np.double(30)
        checksum = np.double(data1 + data2 + data3 + data4 + data5)

        print "socket_lock.acquire() in control_callback()"
        self.socket_lock.acquire()
        print "socket_lock acquired in control_callback()!"
        self.client_socket.sendall(np.int32(48)) # (6 doubles = 6*8 bytes = 48 bytes)

        self.client_socket.sendall(data1)
        self.client_socket.sendall(data2)
        self.client_socket.sendall(data3)
        self.client_socket.sendall(data4)
        self.client_socket.sendall(data5)
        self.client_socket.sendall(checksum)
        self.socket_lock.release()

    def run(self):
        while not rospy.is_shutdown():
            print "####################"
            print "socket_lock.acquire() in run()"
            self.socket_lock.acquire()
            print "socket_lock acquired in run()!"
            data = self.client_socket.recv(4)
            print len(data)
            no_of_bytes_to_read = struct.unpack("<L", data)[0]
            print "number of bytes to read: %d" % no_of_bytes_to_read

            no_of_read_bytes = 0
            while no_of_read_bytes < 8:
                data = self.client_socket.recv(8-no_of_read_bytes)
                no_of_read_bytes += len(data)
                #print data
                #print "***"
            if len(data) == 8:
                message_type = struct.unpack("<d", data)[0]
                print "message type: %f" % message_type
                #print "^^^^^^"
            else:
                message_type = -1 # (error)

            no_of_read_bytes = 0
            if no_of_bytes_to_read - 8 > 0:
                while no_of_read_bytes < no_of_bytes_to_read - 8:
                    data = self.client_socket.recv(no_of_bytes_to_read - 8 - no_of_read_bytes)
                    #print len(data)
                    no_of_read_bytes += len(data)
                    #print data
                    #print "&&&&&&&&&&&&&&&"
            self.socket_lock.release()

            if message_type == 1:
                if len(data) == 128:
                    #print "THE SENSOR DATA HAS ARRIVED!!"
                    us_1 = data[0:8]
                    us_1 = struct.unpack("<d", us_1)[0]
                    #print "us 1: %f" % us_1

                    us_2 = data[8:16]
                    us_2 = struct.unpack("<d", us_2)[0]
                    #print "us 2: %f" % us_2

                    us_3 = data[16:24]
                    us_3 = struct.unpack("<d", us_3)[0]
                    #print "us 3: %f" % us_3

                    us_4 = data[24:32]
                    us_4 = struct.unpack("<d", us_4)[0]
                    #print "us 4: %f" % us_4

                    gyro_1 = data[32:40]
                    gyro_1 = struct.unpack("<d", gyro_1)[0]
                    #print "gyro 1: %f" % gyro_1

                    gyro_2 = data[40:48]
                    gyro_2 = struct.unpack("<d", gyro_2)[0]
                    #print "gyro 2: %f" % gyro_2

                    gyro_3 = data[48:56]
                    gyro_3 = struct.unpack("<d", gyro_3)[0]
                    #print "gyro 3: %f" % gyro_3

                    acc_1 = data[56:64]
                    acc_1 = struct.unpack("<d", acc_1)[0]
                    #print "acc 1: %f" % acc_1

                    acc_2 = data[64:72]
                    acc_2 = struct.unpack("<d", acc_2)[0]
                    #print "acc 2: %f" % acc_2

                    acc_3 = data[72:80]
                    acc_3 = struct.unpack("<d", acc_3)[0]
                    #print "acc 3: %f" % acc_3

                    magn_1 = data[80:88]
                    magn_1 = struct.unpack("<d", magn_1)[0]
                    #print "magn 1: %f" % magn_1

                    magn_2 = data[88:96]
                    magn_2 = struct.unpack("<d", magn_2)[0]
                    #print "magn 2: %f" % magn_2

                    magn_3 = data[96:104]
                    magn_3 = struct.unpack("<d", magn_3)[0]
                    #print "magn 3: %f" % magn_3

                    odoRight = data[104:112]
                    odoRight = struct.unpack("<d", odoRight)[0]
                    #print "odoRight: %f" % odoRight

                    odoLeft = data[112:120]
                    odoLeft = struct.unpack("<d", odoLeft)[0]
                    #print "odoLeft: %f" % odoLeft

                    time = data[120:128]
                    time = struct.unpack("<d", time)[0]
                    #print "time: %f" % time

                    msg = Float64MultiArray()
                    data = [odoRight, odoLeft]
                    #print data
                    msg.data = data
                    self.encoder_pub.publish(msg)

if __name__ == "__main__":
    # create a Communicator object (this will run its __init__ function):
    communicator = Communicator()

    communicator.run()
