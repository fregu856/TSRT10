#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <algorithm>

#include <time.h>

typedef Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MapMatrix;
typedef Eigen::Matrix<int8_t, Eigen::Dynamic, 1> MapEigenVector;

class SlamVisited
{
public:
    SlamVisited();

private:
    void marker_callback_(const visualization_msgs::Marker &msg_obj);
    void map_callback_(const nav_msgs::OccupancyGrid &msg_obj);

    ros::NodeHandle nh_;

    ros::Publisher map_visited_pub_;

    ros::Subscriber map_sub_;
    ros::Subscriber marker_sub_;

    MapMatrix map_matrix_;
    std::vector<double> map_origin_;
    double map_resolution_;
}; // Class SlamVisited

SlamVisited::SlamVisited()
{
    map_origin_.push_back(-1000);
    map_origin_.push_back(-1000);

    map_visited_pub_ =
          nh_.advertise<nav_msgs::OccupancyGrid>("/map_visited", 10);

    map_sub_ = nh_.subscribe("/map", 10, &SlamVisited::map_callback_, this);

    marker_sub_ = nh_.subscribe("/Mapper/vertices", 10,
          &SlamVisited::marker_callback_, this);
}

void SlamVisited::marker_callback_(const visualization_msgs::Marker &msg_obj)
{
    //const clock_t begin_time = clock();

    if (map_origin_[0] != -1000)
    {
        std::cout << "marker_callback_" << std::endl;

        // TODO! protect with mutex
        MapMatrix map_visited_matrix = map_matrix_;
        std::vector<double> map_origin = map_origin_;
        double map_resolution = map_resolution_;

        int cols = map_visited_matrix.cols();
        int rows = map_visited_matrix.rows();

        //std::cout << "rows: " << rows << " cols: " << cols << std::endl;

        std::vector<geometry_msgs::Point> points(msg_obj.points);
        for (int i = 0; i < points.size(); ++i)
        {
            double x = points[i].x;
            double y = points[i].y;

            double x_map = x - map_origin[0];
            double y_map = y - map_origin[1];

            int x_map_ind(x_map/map_resolution); // (col)
            int y_map_ind(y_map/map_resolution); // (row)

            //std::cout << "x_map: " << x_map << " x_map_ind: " << x_map_ind << std::endl;
            //std::cout << "y_map: " << y_map << " y_map_ind: " << y_map_ind << std::endl;

            int col_ind_max = std::min(x_map_ind+10, cols);
            int col_ind_min = std::max(x_map_ind-10, 0);
            int row_ind_max = std::min(y_map_ind+10, rows);
            int row_ind_min = std::max(y_map_ind-10, 0);
            int block_width = col_ind_max - col_ind_min;
            int block_height = row_ind_max - row_ind_min;
            //std::cout << "block_width: " << block_width << " block_height: " << block_height << std::endl;

            MapMatrix visited_block = MapMatrix::Constant(block_height,
                  block_width, -2);
            map_visited_matrix.block(row_ind_min, col_ind_min, block_height,
                  block_width) = visited_block;
        }

        MapEigenVector map_visited_eigen_vector(Eigen::Map<MapEigenVector>(
              map_visited_matrix.data(), cols*rows));
        std::vector<int8_t> map_visited_vector(map_visited_eigen_vector.data(),
              map_visited_eigen_vector.data() + map_visited_eigen_vector.size());

        nav_msgs::OccupancyGrid map_visited_msg;
        map_visited_msg.data = map_visited_vector;
        map_visited_msg.info.resolution = map_resolution;
        map_visited_msg.info.width = cols;
        map_visited_msg.info.height = rows;
        map_visited_msg.info.origin.position.x = map_origin[0];
        map_visited_msg.info.origin.position.y = map_origin[1];
        map_visited_pub_.publish(map_visited_msg);
    }

    //std::cout << float(clock() - begin_time)/CLOCKS_PER_SEC;
}

void SlamVisited::map_callback_(const nav_msgs::OccupancyGrid &msg_obj)
{
    // TODO! protect with mutex
    map_origin_[0] = msg_obj.info.origin.position.x;
    map_origin_[1] = msg_obj.info.origin.position.y;
    map_resolution_ = msg_obj.info.resolution;

    int map_height = msg_obj.info.height;
    int map_width = msg_obj.info.width;

    // std::cout << map_height << std::endl;
    // std::cout << map_width << std::endl;
    // std::cout << map_resolution_ << std::endl;

    std::vector<int8_t> map_data(msg_obj.data);
    // TODO! protect with mutex
    map_matrix_ = Eigen::Map<MapMatrix>(&map_data[0], map_height, map_width);

    // std::cout << "rows: " << map_matrix_.rows() << std::endl;
    // std::cout << "cols: " << map_matrix_.cols() << std::endl;
}

int main(int argc, char **argv)
{
    // initialize this code as a ROS node named slam_visited_cpp_node:
    ros::init(argc, argv, "slam_visited_cpp_node");

    SlamVisited slam_visited;

    ros::spin();

    return 0;
}
