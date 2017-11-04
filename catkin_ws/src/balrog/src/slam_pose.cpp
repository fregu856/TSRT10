#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <vector>
#include <iostream>

int main(int argc, char **argv)
{
    // initialize this code as a ROS node named slam_pose_cpp_node:
    ros::init(argc, argv, "slam_pose_cpp_node");

    ros::NodeHandle nh;

    ros::Publisher pose_pub =
          nh.advertise<std_msgs::Float64MultiArray>("/estimated_pose", 10);

    tf::TransformListener listener;

    ros::Rate rate(10);

    while (ros::ok())
    {
         try
         {
            tf::StampedTransform transform;
            listener.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);

            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();

            tf::Quaternion quaternion = transform.getRotation();
            tf::Matrix3x3 rotation_matrix(quaternion);
            double roll, pitch, yaw;
            rotation_matrix.getRPY(roll, pitch, yaw);
            double theta = yaw;

            std::vector<double> pose;
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(theta);

            std_msgs::Float64MultiArray msg;
            msg.data = pose;
            pose_pub.publish(msg);
         }

        catch (tf::TransformException ex)
        {
            std::cout << "Could not look up transform /map -> /base_footprint!" << std::endl;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
