#!/usr/bin/env python

import socket
import numpy as np
import struct
import time

if __name__ == "__main__":
    # create a TCP/IP socket:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # connect to the RPI server:
    host = "192.168.137.3" # (RPI IP address)
    port = 2100
    client_socket.connect((host, port))

    sum = 1
    print "Start"

    client_socket.sendall(np.int32(48)) # (6 doubles = 6*8 bytes = 48 bytes)

    data1 = np.double(sum)
    data2 = np.double(sum+1)
    data3 = np.double(sum+2)
    data4 = np.double(sum+3)
    data5 = np.double(sum+4)
    checksum = np.double(data1 + data2 + data3 + data4 + data5)

    client_socket.sendall(data1)
    client_socket.sendall(data2)
    client_socket.sendall(data3)
    client_socket.sendall(data4)
    client_socket.sendall(data5)
    client_socket.sendall(checksum)

    print "Done"

    while True:
        print client_socket.recv(1)
        sum += 1
        print sum
        time.sleep(1)
