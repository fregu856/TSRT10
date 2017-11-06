#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf/transform_broadcaster.h>

#include <vector>
#include <iostream>
#include <math.h>

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
          nh_.advertise<std_msgs::Float64MultiArray>("/odom_pose2", 10);

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

    double delta_theta_l = odoLeft;
    double delta_theta_r = odoRight;

    double delta_s_l = delta_theta_l*r_;
    double delta_s_r = delta_theta_r*r_;

    double delta_theta = (delta_s_r - delta_s_l)/b_;

    double delta_s = (delta_s_l + delta_s_r)/2.0;

    x_ = x_ + delta_s*cos(theta_ + delta_theta/2.0);
    y_ = y_ + delta_s*sin(theta_ + delta_theta/2.0);
    theta_ = theta_ + delta_theta;

    theta_ = wrap_to_pi_(theta_);

    std::vector<double> state;
    state.push_back(x_);
    state.push_back(y_);
    state.push_back(theta_);

    std_msgs::Float64MultiArray odom_msg;
    odom_msg.data = state;
    odom_pose_pub_.publish(odom_msg);

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_, y_, 0.0));
    tf::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, theta_);
    transform.setRotation(quaternion);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
          "base_footprint", "odom"));
}

double SlamOdom::wrap_to_pi_(double angle)
{
    const double pi = 3.1415926535897;

    double output;

    if (angle > 0)
    {
        output = fmod(angle + pi, 2*pi) - pi;
    }
    else
    {
        output = -(fmod(-angle + pi, 2*pi) - pi);
    }

    return output;
}

int main(int argc, char **argv)
{
    // initialize this code as a ROS node named slam_odom_cpp_node:
    ros::init(argc, argv, "slam_odom_cpp_node");

    SlamOdom slam_odom;

    ros::spin();

    return 0;
}
