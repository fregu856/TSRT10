// #include <ros/ros.h>
//
// #include <nav2d_karto/MultiMapper.h>
// #include <nav2d_karto/SpaSolver.h>
// #include <nav2d_karto/SpaSolver.h>
//
// ////////////////////////////////
// #include "OpenKarto/OpenMapper.h"
// #include <Eigen/Eigen>
// #include <iostream>
// #include <fstream>
// #include <stdlib.h>
// //////////////////////////////
//
//
// int main(int argc, char **argv)
// {
// 	// Initialize ROS
// 	ros::init(argc, argv, "MultiMapper");
// 	ros::NodeHandle node;
//
// 	// Create a scan-solver
// 	SpaSolver* solver = new SpaSolver();
//
// 	std::ifstream infile("/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/data/graph.g2o");
// 	std::string line;
// 	while (std::getline(infile, line))
// 	{
// 		std::cout << "Test\n";
//
// 		std::stringstream linestream(line);
// 		std::string type;
// 		linestream >> type;
//
// 		if (type.compare("VERTEX_SE2") == 0)
// 		{
// 			int node_id;
// 			double x, y, theta;
// 			linestream >> node_id >> x >> y >> theta;
// 			Eigen::Vector3d vector(x, y, theta);
// 			solver->AddNode2(vector, node_id);
// 		}
//
// 		else if (type.compare("EDGE_SE2") == 0)
// 		{
// 			int source_id, target_id;
// 			Eigen::Matrix<double,3,3> m;
// 			double m_0_0, m_0_1, m_0_2, m_1_1, m_1_2, m_2_2, x, y, theta;
//
// 			linestream >> source_id >> target_id >> x >> y >> theta;
// 			linestream >> m_0_0 >> m_0_1 >> m_0_2 >> m_1_1 >> m_1_2 >> m_2_2;
//
// 			Eigen::Vector3d mean(x, y, theta);
//
// 			m(0,0) = m_0_0;
// 			m(0,1) = m(1,0) = m_0_1;
// 			m(0,2) = m(2,0) = m_0_2;
// 			m(1,1) = m_1_1;
// 			m(1,2) = m(2,1) = m_1_2;
// 			m(2,2) = m_2_2;
//
// 			solver->AddConstraint2(source_id, target_id, mean, m);
// 		}
// 	}
//
// 	std::cout << "Test before\n";
// 	solver->Compute2();
// 	std::cout << "Test after\n";
//
// 	// Quit
// 	return 0;
// }




#include <ros/ros.h>

#include <nav2d_karto/MultiMapper.h>
#include <nav2d_karto/SpaSolver.h>
#include <nav2d_karto/SpaSolver.h>

int main(int argc, char **argv)
{
	// Initialize ROS
	ros::init(argc, argv, "MultiMapper");
	ros::NodeHandle node;

	// Create a scan-solver
	SpaSolver* solver = new SpaSolver();

	// Create the MultiMapper
	MultiMapper mapper;
	mapper.setScanSolver(solver);

	// Start main loop
	ros::Rate publishRate(10);
	while(ros::ok())
	{
		mapper.publishTransform();
		ros::spinOnce();
		publishRate.sleep();
	}

	// Quit
	return 0;
}
