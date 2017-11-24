// This code looks up the transform from /map to /base_footprint at 10 Hz in
// order to get the most recent estimate of the robot's pose [x, y, theta], as
// outputted by SLAM. It also publishes this pose estimate on the ROS topic
// /estimated_pose.

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

    // create a NodeHandle object:
    ros::NodeHandle nh;

    // create a publisher for publishing the estimated robot pose [x, y, theta]
    // on the /estimated_pose ROS topic:
    ros::Publisher pose_pub =
          nh.advertise<std_msgs::Float64MultiArray>("/estimated_pose", 10);

    // create a transform listener object, to be able to look up the transform
    // from /map to /base_footprint (this gives the estimated robot pose):
    tf::TransformListener listener;

    // specify the desired loop frequency to be 10 Hz:
    ros::Rate rate(10);

    while (ros::ok())
    {
         try
         {
            // look up the transform from /map to /base_footprint to get the
            // estimated robot pose:
            tf::StampedTransform transform;
            listener.lookupTransform("map", "base_footprint", ros::Time(0), transform);

            // get the estimated x and y values:
            double x = transform.getOrigin().x();
            double y = transform.getOrigin().y();

            // get the estimated theta value:
            // // get the estimated quaternion:
            tf::Quaternion quaternion = transform.getRotation();
            // // transform the quaternion to a rotation matrix:
            tf::Matrix3x3 rotation_matrix(quaternion);
            // // transform the rotation matrix to roll, pith, yaw (= theta):
            double roll, pitch, yaw;
            rotation_matrix.getRPY(roll, pitch, yaw);
            double theta = yaw;

            // create a pose vector ([x, y, theta]):
            std::vector<double> pose;
            pose.push_back(x);
            pose.push_back(y);
            pose.push_back(theta);

            // publish the pose:
            std_msgs::Float64MultiArray msg;
            msg.data = pose;
            pose_pub.publish(msg);
         }

        catch (tf::TransformException exception)
        {
            std::cout << "Could not look up transform map -> base_footprint!" << std::endl;
        }

        // spin to enable automatic reading of new data published on the
        // subscribed topics:
        ros::spinOnce();

        // sleep to get the desired loop frequency of 10 Hz:
        rate.sleep();
    }

    return 0;
}
