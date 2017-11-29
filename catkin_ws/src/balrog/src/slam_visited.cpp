// This code subscribes to the ROS topics /map (map outputted by the SLAM node)
// and /Mapper/vertices (sampled points [x, y] along the estimated path that the
// robot has traveled, outputted by the SLAM node). Each grid cell that lies
// within a 1x1 m square of any of these points are marked as "visited" in the
// map_visited map, which is published on the /map_visited ROS topic.

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <algorithm>
#include <mutex>

#include <time.h>

typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MapMatrix;
typedef Eigen::Matrix<int8_t, Eigen::Dynamic, 1> MapEigenVector;

class SlamVisited
{
public:
    SlamVisited();

private:
    // callback function for the /Mapper/vertices ROS topic:
    void MarkerCallback(const visualization_msgs::Marker &msg_obj);
    // callback function for the /map ROS topic:
    void MapCallback(const nav_msgs::OccupancyGrid &msg_obj);

    ros::NodeHandle nh_;

    // publisher for publishing the map_visited map:
    ros::Publisher map_visited_pub_;

    // subscriber for the /map ROS topic:
    ros::Subscriber map_sub_;
    // subscriber for the /Mapper/vertices ROS topic:
    ros::Subscriber marker_sub_;

    // map variables:
    MapMatrix map_matrix_;
    std::vector<double> map_origin_;
    double map_resolution_;

    // mutex for protecting the shared map variables:
    std::mutex mutex_;
}; // class SlamVisited

SlamVisited::SlamVisited()
{
    // initialize the map origin:
    map_origin_.push_back(-10000);
    map_origin_.push_back(-10000);

    // initialize the publisher for publishing the map_visited map on the
    // /map_visited ROS topic:
    map_visited_pub_ =
          nh_.advertise<nav_msgs::OccupancyGrid>("/map_visited", 10);

    // initialize the subscriber for the /map ROS topic (everytime a
    // new message is published on the topic, SlamVisited::MapCallback will
    // be called):
    map_sub_ = nh_.subscribe("/map", 10, &SlamVisited::MapCallback, this);

    // initialize the subscriber for the /Mapper/vertices ROS topic (everytime a
    // new message is published on the topic, SlamVisited::MarkerCallback will
    // be called):
    marker_sub_ = nh_.subscribe("/Mapper/vertices", 10,
          &SlamVisited::MarkerCallback, this);
}

// callback function for the /Mapper/vertices topic (sampled points [x, y] along
// the estimated path that the robot has traveled). Each grid cell that lies
// within a 1x1 m square of any of these points are marked as "visited" in the
// map_visited map, which is published on the /map_visited topic:
void SlamVisited::MarkerCallback(const visualization_msgs::Marker &msg_obj)
{
    //const clock_t begin_time = clock();

    if (map_origin_[0] != -10000) // (if at least one map has been received)
    {
        std::cout << "MarkerCallback" << std::endl;

        // read the shared map variables:
        mutex_.lock();
        MapMatrix map_visited_matrix = map_matrix_;
        std::vector<double> map_origin = map_origin_;
        double map_resolution = map_resolution_;
        mutex_.unlock();

        // get the size of the map matrix:
        int cols = map_visited_matrix.cols();
        int rows = map_visited_matrix.rows();

        // create a vector of all sampled points along the robot path:
        std::vector<geometry_msgs::Point> points(msg_obj.points);

        // mark each grid cell that lies within a 1x1 m square of any point as
        // "visited" (= -2) in map_visited_matrix:
        for (int i = 0; i < points.size(); ++i) // (for each point in points:)
        {
            // get the x, y coordinates of the point:
            double x = points[i].x;
            double y = points[i].y;

            // transform the x, y coordinates to corresponding map matrix indices:
            double x_map = x - map_origin[0];
            double y_map = y - map_origin[1];
            int x_map_ind(x_map/map_resolution); // (col)
            int y_map_ind(y_map/map_resolution); // (row)

            // compute the corner indices of the 1x1 m square centered at the
            // point, making sure we never try to access out-of-bounds elements:
            int col_ind_max = std::min(x_map_ind+10, cols);
            int col_ind_min = std::max(x_map_ind-10, 0);
            int row_ind_max = std::min(y_map_ind+10, rows);
            int row_ind_min = std::max(y_map_ind-10, 0);

            // compute the size of the resulting rectangle:
            int block_width = col_ind_max - col_ind_min;
            int block_height = row_ind_max - row_ind_min;

            // create a block of -2s (visited cells):
            MapMatrix visited_block = MapMatrix::Constant(block_height,
                  block_width, -2);

            // insert the block of -2s in map_visited_matrix, centered at the
            // current point:
            map_visited_matrix.block(row_ind_min, col_ind_min, block_height,
                  block_width) = visited_block;
        }

        // convert map_visited_matrix into a MapEigenVector:
        MapEigenVector map_visited_eigen_vector(Eigen::Map<MapEigenVector>(
              map_visited_matrix.data(), cols*rows));

        // convert map_visited_eigen_vector into a vector:
        std::vector<int8_t> map_visited_vector(map_visited_eigen_vector.data(),
              map_visited_eigen_vector.data() + map_visited_eigen_vector.size());

        // create an OccupancyGrid message of the resulting map_visited map:
        nav_msgs::OccupancyGrid map_visited_msg;
        map_visited_msg.data = map_visited_vector;
        map_visited_msg.info.resolution = map_resolution;
        map_visited_msg.info.width = cols;
        map_visited_msg.info.height = rows;
        map_visited_msg.info.origin.position.x = map_origin[0];
        map_visited_msg.info.origin.position.y = map_origin[1];

        // publish the message on the /map_visited topic:
        map_visited_pub_.publish(map_visited_msg);
    }

    //std::cout << float(clock() - begin_time)/CLOCKS_PER_SEC;
}

// callback function for the /map topic. The map data is converted into a matrix
// and is saved together with map metadata in shared variables:
void SlamVisited::MapCallback(const nav_msgs::OccupancyGrid &msg_obj)
{
    // read the size of the received map:
    int map_height = msg_obj.info.height;
    int map_width = msg_obj.info.width;

    // read the received map data into a vector:
    std::vector<int8_t> map_data(msg_obj.data);

    // save the map data and metadata in shared variables:
    mutex_.lock();
    map_origin_[0] = msg_obj.info.origin.position.x;
    map_origin_[1] = msg_obj.info.origin.position.y;
    map_resolution_ = msg_obj.info.resolution;
    // // convert the vector of map data into a matrix:
    map_matrix_ = Eigen::Map<MapMatrix>(&map_data[0], map_height, map_width);
    mutex_.unlock();
}

int main(int argc, char **argv)
{
    // initialize this code as a ROS node named slam_visited_cpp_node:
    ros::init(argc, argv, "slam_visited_cpp_node");

    // create a SlamVisited object:
    SlamVisited slam_visited;

    // spin to enable automatic reading of new data published on the
    // subscribed topics (while the ROS node is running, we will not exit this
    // function):
    ros::spin();

    return 0;
}
