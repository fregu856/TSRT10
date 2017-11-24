// This code communicates with the RPI on Balrog using a TCP socket. It
// subscribes to the ROS topic /control_signals and forwards this data (outputted
// by the controller) to the RPI. It also continuously reads data transmitted by
// the RPI, and forwards the wheel encoder data by publishing this on the
// /encoder_data ROS topic.

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>
#include <vector>
#include <mutex>

#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>

typedef unsigned char Byte;

class Communicator
{
public:
    Communicator(int tcp_socket);

    // function for continuously reading the data transmitted by the RPI:
    void run();

private:
    // callback function for the /control_signals ROS topic:
    void control_callback_(const std_msgs::Float64MultiArray &msg);

    ros::NodeHandle nh_;

    // publisher for publishing the wheel encoder data:
    ros::Publisher encoder_pub_;

    // subscriber for the /control_signals ROS topic:
    ros::Subscriber control_sub_;

    // descriptor/id of TCP socket used to communicate with the RPI:
    int tcp_socket_;

    // mutex for protecting the shared TCP socket:
    std::mutex mutex_;
}; // class Communicator

Communicator::Communicator(int tcp_socket)
{
    tcp_socket_ = tcp_socket;

    // initialize the publisher for publishing the wheel encoder data on the
    // /encoder_data ROS topic:
    encoder_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>("/encoder_data", 10);

    // initialize the subscriber for the /control_signals ROS topic (everytime a
    // new message is published on the topic, Communicator::control_callback_
    // will be called):
    control_sub_ = nh_.subscribe("/control_signals", 10,
          &Communicator::control_callback_, this);

    // initialize the RPI communication by transmitting a [0, 0] control signal:
    double omega_l = 0;
    double omega_r = 0;
    double data1 = omega_l;
    double data2 = omega_r;
    double data3 = 10;
    double data4 = 20;
    double data5 = 30;
    double checksum = data1 + data2 + data3 + data4 + data5;
    int msg_size = 48; // (6 doubles = 6*8 = 48 bytes)
    Byte msg_size_bytes[4]; // (an int is 4 bytes)
    memcpy(msg_size_bytes, &msg_size, sizeof(msg_size));
    double data[6] = {data1, data2, data3, data4, data5, checksum};
    Byte data_bytes[48];
    memcpy(data_bytes, &data, sizeof(data));
    mutex_.lock();
    send(tcp_socket_, msg_size_bytes, 4, 0);
    send(tcp_socket_, data_bytes, 48, 0);
    mutex_.unlock();
}

// function for continuously reading the data transmitted by the RPI. It also
// forwards the wheel encoder data by publishing this on the /encoder_data topic:
void Communicator::run()
{
    while (ros::ok())
    {
        // (from above: typedef unsigned char Byte)

        Byte size_of_data_bytes[4];
        Byte message_type_bytes[8];
        Byte odoRight_bytes[8];
        Byte odoLeft_bytes[8];
        Byte checksum_bytes[8];

        int read_bytes_total;
        int read_bytes;

        // read all data transmitted from the RPI from the shared TCP socket:
        mutex_.lock();
        // // read the int (4 bytes) containing the size (number of bytes) of
        // // the message sent by the RPI:
        read_bytes_total = 0;
        while (read_bytes_total < 4)
        {
            read_bytes = read(tcp_socket_, size_of_data_bytes + read_bytes_total, 4 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        // // read the double (8 bytes) containing the message type of the
        // // message sent by the RPI:
        read_bytes_total = 0;
        while (read_bytes_total < 8)
        {
            read_bytes = read(tcp_socket_, message_type_bytes + read_bytes_total, 8 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        // // read the double (8 bytes) containing (if message_type == 1,
        // // otherwise this double just contains 'void data') odoRight:
        read_bytes_total = 0;
        while (read_bytes_total < 8)
        {
            read_bytes = read(tcp_socket_, odoRight_bytes + read_bytes_total, 8 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        // // read the double (8 bytes) containing (if message_type == 1,
        // // otherwise this double just contains 'void data') odoLeft:
        read_bytes_total = 0;
        while (read_bytes_total < 8)
        {
            read_bytes = read(tcp_socket_, odoLeft_bytes + read_bytes_total, 8 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        // // read the double (8 bytes) containing (if message_type == 1,
        // // otherwise this double just contains 'void data') the message checksum:
        read_bytes_total = 0;
        while (read_bytes_total < 8)
        {
            read_bytes = read(tcp_socket_, checksum_bytes + read_bytes_total, 8 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        mutex_.unlock();

        // convert size_of_data_bytes (array of Bytes) into an int:
        int size_of_data;
        memcpy(&size_of_data, size_of_data_bytes, sizeof(size_of_data));

        if (size_of_data == 32)
        {
            // convert message_type_bytes (array of Bytes) into a double:
            double message_type;
            memcpy(&message_type, message_type_bytes, sizeof(message_type));
            std::cout << message_type << std::endl;

            if (message_type == 1) // (if sensor data:)
            {
                // convert odoRight_bytes (array of Bytes) into a double:
                double odoRight;
                memcpy(&odoRight, odoRight_bytes, sizeof(odoRight));

                // convert odoLeft_bytes (array of Bytes) into a double:
                double odoLeft;
                memcpy(&odoLeft, odoLeft_bytes, sizeof(odoLeft));

                // convert checksum_bytes (array of Bytes) into a double:
                double checksum;
                memcpy(&checksum, checksum_bytes, sizeof(checksum));

                // compute checksum:
                double computed_checksum = odoRight + odoLeft;

                if (computed_checksum == checksum)
                {
                    // create a vector of the encoder data:
                    std::vector<double> encoder_data;
                    encoder_data.push_back(odoRight);
                    encoder_data.push_back(odoLeft);

                    // publish the encoder_data on the /encoder_data topic:
                    std_msgs::Float64MultiArray encoder_msg;
                    encoder_msg.data = encoder_data;
                    encoder_pub_.publish(encoder_msg);
                }
                else
                {
                    std::cout << "#############################################" << std::endl;
                    std::cout << "#############################################" << std::endl;
                    std::cout << "#############################################" << std::endl;
                    std::cout << "checksums don't match!" << std::endl;
                    std::cout << "#############################################" << std::endl;
                    std::cout << "#############################################" << std::endl;
                    std::cout << "#############################################" << std::endl;
                }
            }
            else if (message_type == 0)
            {
                std::cout << "ping received!" << std::endl;
            }
            else
            {
                std::cout << "#############################################" << std::endl;
                std::cout << "#############################################" << std::endl;
                std::cout << "#############################################" << std::endl;
                std::cout << "unknown message type!" << std::endl;
                std::cout << "#############################################" << std::endl;
                std::cout << "#############################################" << std::endl;
                std::cout << "#############################################" << std::endl;
            }
        }
        else // (if (size_of_data != 32))
        {
            std::cout << "#############################################" << std::endl;
            std::cout << "#############################################" << std::endl;
            std::cout << "#############################################" << std::endl;
            std::cout << "size_of_data != 32, there must be an error in transmission!" << std::endl;
            std::cout << "#############################################" << std::endl;
            std::cout << "#############################################" << std::endl;
            std::cout << "#############################################" << std::endl;
        }

        // spin once to enable automatic reading of new data published on the
        // subscribed topic:
        ros::spinOnce();
    }
}

// callback function for the /control_signals ROS topic. Forwards the data
// published by the controller to the RPI:
void Communicator::control_callback_(const std_msgs::Float64MultiArray &msg_obj)
{
    // read the received control signals into a vector:
    std::vector<double> ctrl_signals(msg_obj.data);

    // get the desired/commanded angular velocity of the left wheel:
    double omega_l = ctrl_signals[0];
    // get the desired/commanded angular velocity of the right wheel:
    double omega_r = ctrl_signals[1];

    // the RPI can only receive messages containing 6 doubles, we send omega_l
    // and omega_r + three control values + a checksum:
    double data1 = omega_l;
    double data2 = omega_r;
    double data3 = 10;
    double data4 = 20;
    double data5 = 30;
    double checksum = data1 + data2 + data3 + data4 + data5;

    // state the size (number of bytes) of the message to be transmitted and
    // convert it into an array of Bytes:
    int msg_size = 48; // (6 doubles = 6*8 = 48 bytes)
    Byte msg_size_bytes[4]; // (an int is 4 bytes)
    memcpy(msg_size_bytes, &msg_size, sizeof(msg_size));

    // put all data to be transmitted in an array of doubles and then convert
    // it into an array of Bytes:
    double data[6] = {data1, data2, data3, data4, data5, checksum};
    Byte data_bytes[48];
    memcpy(data_bytes, &data, sizeof(data));

    // send the message to the RPI using the shared TCP socket:
    mutex_.lock();
    send(tcp_socket_, msg_size_bytes, 4, 0); // (send size of message)
    send(tcp_socket_, data_bytes, 48, 0); // (send message data)
    mutex_.unlock();
}

int main(int argc, char **argv)
{
    // initialize this code as a ROS node named communicator_cpp_node:
    ros::init(argc, argv, "communicator_cpp_node");

    // connect to the RPI TCP socket:
    int tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_socket < 0)
    {
        std::cout << "Could not create socket!" << std::endl;
        return -1;
    }
    struct sockaddr_in server_address;
    memset(&server_address, '0', sizeof(server_address));
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(2100); // (2100 is the port that is open on the RPI)
    inet_pton(AF_INET, "10.0.0.10", &server_address.sin_addr); // (10.0.0.10 is the RPI IP address)
    int connect_status = connect(tcp_socket, (struct sockaddr*) &server_address, sizeof(server_address));
    if (connect_status < 0)
    {
        std::cout << "Connection to the TCP socket failed!" << std::endl;
        return -1;
    }

    // create a Communicator object:
    Communicator communicator(tcp_socket);

    // run the run() member function:
    communicator.run();

    return 0;
}
