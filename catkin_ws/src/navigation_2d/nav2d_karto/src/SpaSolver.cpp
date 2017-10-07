/*
 * Copyright 2010 SRI International
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/console.h>

#include <nav2d_karto/SpaSolver.h>

#include <iostream>
#include <fstream>
#include <stdlib.h>

SpaSolver::SpaSolver()
{
//	m_Spa.verbose = true;
	mLastSPA = ros::Time::now();
}

SpaSolver::~SpaSolver()
{

}

void SpaSolver::Clear()
{
	corrections.Clear();
}

const karto::ScanSolver::IdPoseVector& SpaSolver::GetCorrections() const
{
	return corrections;
}

// // SE-Sync version:
// void SpaSolver::Compute()
// {
// 		std::cout << "Start of SpaSolver::Compute in SpaSolver.cpp\n";
// 		corrections.Clear();
//
// 		std::ifstream infile("/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/MATLAB/examples/test.txt");
// 	  long double x, y, theta;
// 	  int id = 0;
// 	  while (infile >> x >> y >> theta)
// 	  {
// 			//if (id > 0)
// 			//{
// 			//std::cout << id << "\n";
// 			karto::Pose2 pose(x, y, theta);
// 			corrections.Add(karto::Pair<int, karto::Pose2>(id, pose));
// 			//}
//
//     	++id;
// 	  }
//
// 		mLastSPA = ros::Time::now();
// 		std::cout << "End of SpaSolver::Compute in SpaSolver.cpp\n";
// }

// SPA version:
void SpaSolver::Compute()
{
	ros::Duration d = ros::Time::now() - mLastSPA;
	if(d.toSec() > 5)
	{
		std::cout << "Start of SpaSolver::Compute in SpaSolver.cpp\n";
		corrections.Clear();
		typedef std::vector<Node2d, Eigen::aligned_allocator<Node2d> > NodeVector;

		ROS_INFO("Calling doSPA for loop closure");
		m_Spa.doSPA(40);
		ROS_INFO("Finished doSPA for loop closure");
		NodeVector nodes = m_Spa.getNodes();
		forEach(NodeVector, &nodes)
		{
			karto::Pose2 pose(iter->trans(0), iter->trans(1), iter->arot);
			corrections.Add(karto::Pair<int, karto::Pose2>(iter->nodeId, pose));
		}
		mLastSPA = ros::Time::now();
		std::cout << "End of SpaSolver::Compute in SpaSolver.cpp\n";
	}
}

void SpaSolver::Compute2()
{
	std::cout << "Start of SpaSolver::Compute in SpaSolver.cpp\n";
	corrections.Clear();
	typedef std::vector<Node2d, Eigen::aligned_allocator<Node2d> > NodeVector;

	ROS_INFO("Calling doSPA for loop closure");
	ros::Time time_before = ros::Time::now();
	m_Spa.doSPA(40);
	ros::Duration compute_time = ros::Time::now() - time_before;
	std::cout << compute_time << "\n";
	ROS_INFO("Finished doSPA for loop closure");
	NodeVector nodes = m_Spa.getNodes();

	std::ofstream node_file;
	node_file.open ("/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/data/graph.txt", std::ios_base::app);

	forEach(NodeVector, &nodes)
	{
		std::cout << "Adding line to solution file\n";
	  node_file << iter->nodeId << " " << iter->trans(0);
		node_file << " " << iter->trans(1) << " " << iter->arot << "\n";
	}
	node_file.close();
	std::cout << "End of SpaSolver::Compute in SpaSolver.cpp\n";
}

void SpaSolver::reCompute()
{
	ros::Duration d = ros::Time::now() - mLastSPA;
	if(d.toSec() > 15)
		Compute();
}

void SpaSolver::AddNode(karto::Vertex<karto::LocalizedObjectPtr>* pVertex)
{
	karto::Pose2 pose = pVertex->GetVertexObject()->GetCorrectedPose();
	Eigen::Vector3d vector(pose.GetX(), pose.GetY(), pose.GetHeading());
	m_Spa.addNode(vector, pVertex->GetVertexObject()->GetUniqueId());

 	////////// MODIFIED START
	std::ofstream graph_file;
  graph_file.open ("/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/data/graph.g2o", std::ios_base::app);
  graph_file << "VERTEX_SE2" << " " << pVertex->GetVertexObject()->GetUniqueId();
	graph_file << " " << pose.GetX() << " " << pose.GetY() << " ";
	graph_file << pose.GetHeading() << "\n";
  graph_file.close();
	////////// MODIFIED END
}

void SpaSolver::AddNode2(Eigen::Vector3d vector, int id)
{
	std::cout << "AddNode2\n";
	m_Spa.addNode(vector, id);
}

void SpaSolver::AddConstraint(karto::Edge<karto::LocalizedObjectPtr>* pEdge)
{
	karto::LocalizedObjectPtr pSource = pEdge->GetSource()->GetVertexObject();
	karto::LocalizedObjectPtr pTarget = pEdge->GetTarget()->GetVertexObject();
	karto::LinkInfo* pLinkInfo = (karto::LinkInfo*)(pEdge->GetLabel());

	karto::Pose2 diff = pLinkInfo->GetPoseDifference();
	Eigen::Vector3d mean(diff.GetX(), diff.GetY(), diff.GetHeading());

	karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
	Eigen::Matrix<double,3,3> m;
	m(0,0) = precisionMatrix(0,0);
	m(0,1) = m(1,0) = precisionMatrix(0,1);
	m(0,2) = m(2,0) = precisionMatrix(0,2);
	m(1,1) = precisionMatrix(1,1);
	m(1,2) = m(2,1) = precisionMatrix(1,2);
	m(2,2) = precisionMatrix(2,2);

	m_Spa.addConstraint(pSource->GetUniqueId(), pTarget->GetUniqueId(), mean, m);

	////////// MODIFIED START
	std::ofstream graph_file;
  graph_file.open ("/home/fregu856/AA273/AA273_project/catkin_ws/src/SE-Sync/data/graph.g2o", std::ios_base::app);
  graph_file << "EDGE_SE2" << " " << pSource->GetUniqueId();
	graph_file << " " << pTarget->GetUniqueId() << " " << diff.GetX() << " ";
	graph_file << diff.GetY() << " " << diff.GetHeading() << " ";
	graph_file << m(0,0) << " " << m(0,1) << " " << m(0,2) << " " << m(1,1);
	graph_file << " " << m(1,2) << " " << m(2,2) << "\n";
  graph_file.close();
	////////// MODIFIED END
}

void SpaSolver::AddConstraint2(int source_id, int target_id, Eigen::Vector3d mean, Eigen::Matrix<double,3,3> m)
{
	std::cout << "AddConstraint2\n";
	m_Spa.addConstraint(source_id, target_id, mean, m);
}
