// This code subscribes to the ROS topic /encoder_data which contains the most
// recent sensor data for the left and right wheel encoders. This data is
// transformed to an estimate of the robot pose [x, y, theta] (dead reackoning),
// which is published on the ROS topic /odom_pose and broadcasted as a transform
// from /odom to /base_footprint (this is taken as input in the OpenKarto SLAM
// ROS node).

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
    // callback function for the /encoder_data ROS topic:
    void OdomCallback(const std_msgs::Float64MultiArray &msg);

    // helper function to wrap angles to lie in the range [-pi, pi]:
    double WrapToPi(double angle);

    ros::NodeHandle nh_;

    // publisher for publishing the estimated robot pose [x, y, theta]:
    ros::Publisher odom_pose_pub_;

    // subscriber for the /encoder_data ROS topic:
    ros::Subscriber encoder_sub_;

    // broadcaster to broadcast the estimated robot pose as a transform
    // from /odom to /base_footprint:
    tf::TransformBroadcaster broadcaster_;

    // robot pose:
    double x_;
    double y_;
    double theta_;

    // physical constants:
    static constexpr double r_ = 0.09; // (wheel radius)
    static constexpr double b_ = 0.6138; // (wheel base)
}; // class SlamOdom

SlamOdom::SlamOdom()
{
    // initialize the robot pose:
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;

    // initialize the publisher for publishing the estimated robot pose [x, y, theta]
    // on the /odom_pose ROS topic:
    odom_pose_pub_ =
          nh_.advertise<std_msgs::Float64MultiArray>("/odom_pose", 10);

    // initialize the subscriber for the /encoder_data ROS topic (everytime a
    // new message is published on the topic, SlamOdom::OdomCallback will
    // be called):
    encoder_sub_ =
          nh_.subscribe("/encoder_data", 10, &SlamOdom::OdomCallback, this);
}

// callback function for the /encoder_data ROS topic. It transforms the received
// wheel encoder sensor data to an estimate of the robot pose [x, y, theta]
// (dead reackoning), which is published on the ROS topic /odom_pose and
// broadcasted as a transform from /odom to /base_footprint (this is taken as
// input in the OpenKarto SLAM ROS node):
void SlamOdom::OdomCallback(const std_msgs::Float64MultiArray &msg)
{
    // read the received sensor data into a vector:
    std::vector<double> encoder_data(msg.data);

    // get the change in angle of the left and right wheels:
    double odo_right = encoder_data[0];
    double odo_left = encoder_data[1];
    double delta_theta_l = odo_left;
    double delta_theta_r = odo_right;

    // compute the traveled distance for the left and right wheels:
    double delta_s_l = delta_theta_l*r_;
    double delta_s_r = delta_theta_r*r_;

    // compute the change in theta and traveled distance of Balrog:
    double delta_theta = (delta_s_r - delta_s_l)/b_;
    double delta_s = (delta_s_l + delta_s_r)/2.0;

    // update the robot pose [x_, y_, theta_]:
    x_ = x_ + delta_s*cos(theta_ + delta_theta/2.0);
    y_ = y_ + delta_s*sin(theta_ + delta_theta/2.0);
    theta_ = theta_ + delta_theta;

    // wrap theta_ to lie in the range [-pi, pi]:
    theta_ = WrapToPi(theta_);

    // create a pose vector ([x_, y_, theta_]):
    std::vector<double> state;
    state.push_back(x_);
    state.push_back(y_);
    state.push_back(theta_);

    // publish the pose:
    std_msgs::Float64MultiArray odom_msg;
    odom_msg.data = state;
    odom_pose_pub_.publish(odom_msg);

    // broadcast the pose as a transform from /odom to /base_footprint:
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x_, y_, 0.0));
    tf::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, theta_);
    transform.setRotation(quaternion);
    broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
          "odom", "base_footprint"));
}

// helper function to wrap angles to lie in the range [-pi, pi]:
double SlamOdom::WrapToPi(double angle)
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

    // create a SlamOdom object:
    SlamOdom slam_odom;

    // spin to enable automatic reading of new data published on the
    // subscribed topic (while the ROS node is running, we will not exit this
    // function):
    ros::spin();

    return 0;
}
