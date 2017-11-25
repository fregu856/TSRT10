#!/usr/bin/env python

# NOTE! This code is a python version of balrog/src/communicator.cpp and is not
# used. It is however saved for future reference.

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
        self.host = "10.0.0.10" # (RPI IP address)
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
        self.socket_lock.acquire()
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

        #print "socket_lock.acquire() in control_callback()"
        self.socket_lock.acquire()
        #print "socket_lock acquired in control_callback()!"
        self.client_socket.sendall(np.int32(48)) # (6 doubles = 6*8 bytes = 48 bytes)

        self.client_socket.sendall(data1)
        self.client_socket.sendall(data2)
        self.client_socket.sendall(data3)
        self.client_socket.sendall(data4)
        self.client_socket.sendall(data5)
        self.client_socket.sendall(checksum)
        self.socket_lock.release()
        #print "socket_lock released in control_callback()!"

    def run(self):
        while not rospy.is_shutdown():
            no_of_read_bytes_size = 0
            no_of_read_bytes_data = 0
            #print "socket_lock.acquire() in run()"
            self.socket_lock.acquire()
            #print "socket_lock acquired in run()!"
            while no_of_read_bytes_size < 4: # (4 first bytes: no of bytes of data, which always should equal 32)
                size_data = self.client_socket.recv(4 - no_of_read_bytes_size)
                no_of_read_bytes_size += len(size_data)
            while no_of_read_bytes_data < 32: # (32 bytes of data, 4 doubles)
                data = self.client_socket.recv(32 - no_of_read_bytes_data)
                no_of_read_bytes_data += len(data)
            self.socket_lock.release()
            #print "socket_lock released in run()!"

            if len(size_data) == 4 and len(data) == 32:
                no_of_data_bytes = struct.unpack("<L", size_data)[0]

                if no_of_data_bytes == 32:
                    message_type = struct.unpack("<d", data[0:8])[0]
                    print "message_type: %f" % message_type

                    if message_type == 1: # (sensor data)
                        odoRight = struct.unpack("<d", data[8:16])[0]

                        odoLeft = struct.unpack("<d", data[16:24])[0]

                        checksum = struct.unpack("<d", data[24:32])[0]

                        computed_checksum = odoLeft + odoRight

                        if computed_checksum == checksum:
                            msg = Float64MultiArray()
                            data = [odoRight, odoLeft]
                            msg.data = data
                            self.encoder_pub.publish(msg)
                        else:
                            print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                            print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                            print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                            print "checksums don't match!"
                            print "########################################################"
                            print "########################################################"
                            print "########################################################"

                    elif message_type == 0:
                        print "ping received!"

                    else: # (this should never happen)
                        print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                        print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                        print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                        print "unknown message type!"
                        print "########################################################"
                        print "########################################################"
                        print "########################################################"

                else: # (if no_of_data_bytes != 32:)
                    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                    print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                    print "no_of_data_bytes != 32, there must be an error in transmission!"
                    print "########################################################"
                    print "########################################################"
                    print "########################################################"

            else: # (if not (len(size_data) == 4 and len(data) == 32))
                print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                print "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@"
                print "didn't receive all data in one reading!"
                print "########################################################"
                print "########################################################"
                print "########################################################"

if __name__ == "__main__":
    # create a Communicator object (this will run its __init__ function):
    communicator = Communicator()

    communicator.run()
