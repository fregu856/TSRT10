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

    void run();

private:
    void control_callback_(const std_msgs::Float64MultiArray &msg);

    ros::NodeHandle nh_;

    ros::Publisher encoder_pub_;
    ros::Subscriber control_sub_;

    int tcp_socket_;

    std::mutex mutex_;
}; // Class Communicator

Communicator::Communicator(int tcp_socket)
{
    tcp_socket_ = tcp_socket;

    encoder_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>("/encoder_data", 10);

    control_sub_ =
          nh_.subscribe("/control_signals", 10, &Communicator::control_callback_, this);

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

void Communicator::run()
{
    while (ros::ok())
    {
        Byte size_of_data_bytes[4];
        Byte message_type_bytes[8];
        Byte odoRight_bytes[8];
        Byte odoLeft_bytes[8];
        Byte checksum_bytes[8];

        int read_bytes_total;
        int read_bytes;

        mutex_.lock();
        read_bytes_total = 0;
        while (read_bytes_total < 4)
        {
            read_bytes = read(tcp_socket_, size_of_data_bytes + read_bytes_total, 4 - read_bytes_total);
            read_bytes_total += read_bytes;
        }

        read_bytes_total = 0;
        while (read_bytes_total < 8)
        {
            read_bytes = read(tcp_socket_, message_type_bytes + read_bytes_total, 8 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        read_bytes_total = 0;
        while (read_bytes_total < 8)
        {
            read_bytes = read(tcp_socket_, odoRight_bytes + read_bytes_total, 8 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        read_bytes_total = 0;
        while (read_bytes_total < 8)
        {
            read_bytes = read(tcp_socket_, odoLeft_bytes + read_bytes_total, 8 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        read_bytes_total = 0;
        while (read_bytes_total < 8)
        {
            read_bytes = read(tcp_socket_, checksum_bytes + read_bytes_total, 8 - read_bytes_total);
            read_bytes_total += read_bytes;
        }
        mutex_.unlock();

        int size_of_data;
        memcpy(&size_of_data, size_of_data_bytes, sizeof(size_of_data));

        if (size_of_data == 32)
        {
            double message_type;
            memcpy(&message_type, message_type_bytes, sizeof(message_type));
            std::cout << message_type << std::endl;

            if (message_type == 1) // (if sensor data)
            {
                double odoRight;
                memcpy(&odoRight, odoRight_bytes, sizeof(odoRight));

                double odoLeft;
                memcpy(&odoLeft, odoLeft_bytes, sizeof(odoLeft));

                double checksum;
                memcpy(&checksum, checksum_bytes, sizeof(checksum));

                double computed_checksum = odoRight + odoLeft;

                if (computed_checksum == checksum)
                {
                  std::vector<double> encoder_data;
                  encoder_data.push_back(odoRight);
                  encoder_data.push_back(odoLeft);

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

        ros::spinOnce();
    }
}

void Communicator::control_callback_(const std_msgs::Float64MultiArray &msg_obj)
{
    std::vector<double> ctrl_signals(msg_obj.data);

    double omega_l = ctrl_signals[0];
    double omega_r = ctrl_signals[1];

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

int main(int argc, char **argv)
{
    std::cout << "communicator.cpp" << std::endl;

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
    inet_pton(AF_INET, "192.168.137.3", &server_address.sin_addr); // (192.168.137.3 is the RPI IP address)
    int connect_status = connect(tcp_socket, (struct sockaddr *)&server_address, sizeof(server_address));
    if (connect_status < 0)
    {
        std::cout << "Connection to the TCP socket failed!" << std::endl;
        return -1;
    }

    Communicator communicator(tcp_socket);

    communicator.run();

    return 0;
}
