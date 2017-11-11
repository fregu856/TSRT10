#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <iostream>

#include <mutex>

#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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
}

void Communicator::run()
{
    while (ros::ok())
    {
        std::cout << "Hej" << std::endl;

        ros::spinOnce();
    }
}

void Communicator::control_callback_(const std_msgs::Float64MultiArray &msg_obj)
{

}

int main(int argc, char **argv)
{
    std::cout << "communicator.cpp" << std::endl;

    // initialize this code as a ROS node named communicator_cpp_node:
    ros::init(argc, argv, "communicator_cpp_node");

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
