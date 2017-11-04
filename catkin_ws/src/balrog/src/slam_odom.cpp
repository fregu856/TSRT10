#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf/transform_broadcaster.h>

#include <vector>
#include <iostream>

class SlamOdom
{
public:
    SlamOdom();

private:
    void odom_callback_(const std_msgs::Float64MultiArray &msg);

    double wrap_to_pi_(double angle);

    ros::NodeHandle nh_;

    ros::Publisher odom_pose_pub_;
    ros::Subscriber encoder_sub_;

    tf::TransformBroadcaster broadcaster_;

    double x_;
    double y_;
    double theta_;

    static const double r_ = 0.09;
    static const double b_ = 0.6138;
}; // Class SlamOdom

SlamOdom::SlamOdom()
{
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    odom_pose_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>("/odom_pose", 10);

    encoder_sub_ =
          nh_.subscribe("/encoder_data", 10, &SlamOdom::odom_callback_, this);
}

void SlamOdom::odom_callback_(const std_msgs::Float64MultiArray &msg)
{
    std::vector<double> encoder_data(msg.data);

    double odoRight = encoder_data[0];
    double odoLeft = encoder_data[1];

    std::cout << "odoRight: " << odoRight << std::endl;
    std::cout << "odoLeft: " << odoLeft << std::endl;

    // FORTSATT KODA HAR!s
}

double SlamOdom::wrap_to_pi_(double angle)
{
    int i = 0;
}

int main(int argc, char **argv)
{
    // initialize this code as a ROS node named slam_odom_cpp_node:
    ros::init(argc, argv, "slam_odom_cpp_node");

    SlamOdom slam_odom;

    ros::spin();

    return 0;
}
