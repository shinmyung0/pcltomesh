
#pragma once

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include <vector>
#include "shared_edge.h"
#include "util.h"
#include "shared_triangle.h"

class BallPivoter
{
public:
	BallPivoter(const pcl::PointCloud<pcl::PointNormal>::Ptr &_cloud, const double _ballRadius);
	~BallPivoter();

	std::pair<int, SharedTrianglePtr> pivot(const MeshedEdgePtr &_edge);
	SharedTrianglePtr findSeed();

	inline pcl::PointNormal *getPoint(const int _index) const
	{
		return &cloud->at(_index);
	}

	inline bool isUsed(const int _index) const
	{
		return notUsed.find(_index) == notUsed.end();
	}

	inline void setUsed(const int _index)
	{
		notUsed.erase(_index);
		notUsedArray[_index] = false;
	}

private:
	std::pair<Eigen::Vector3f, double> getCircumscribedCircle(const Eigen::Vector3f &_p0, const Eigen::Vector3f &_p1, const Eigen::Vector3f &_p2) const;
	bool getBallCenter(const int _index0, const int _index1, const int _index2, Eigen::Vector3f &_center, Eigen::Vector3i &_sequence) const;

	bool isEmpty(const std::vector<int> &_data, const int _index0, const int _index1, const int _index2, const Eigen::Vector3f &_ballCenter) const;
	std::vector<int> getNeighbors(const pcl::PointNormal &_point, const double _radius) const;

	pcl::KdTreeFLANN<pcl::PointNormal> kdtree;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
	std::map<int, bool> notUsed;
	double ballRadius;

	bool *notUsedArray;
};
