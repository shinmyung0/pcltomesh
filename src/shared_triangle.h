#pragma once

#include <pcl/common/common.h>
#include <vector>
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "shared_edge.h"

// Triangle smart pointer
class MeshedTriangle;
typedef boost::shared_ptr<MeshedTriangle> SharedTrianglePtr;

class MeshedTriangle
{
public:
	MeshedTriangle();
	MeshedTriangle(const pcl::PointNormal &_v0, const pcl::PointNormal &_v1, const pcl::PointNormal &_v2, const int _index0, const int _index1, const int _index2, const pcl::PointNormal &_ballCenter, const double _ballRadius);
	MeshedTriangle(pcl::PointNormal *_v0, pcl::PointNormal *_v1, pcl::PointNormal *_v2, const int _index0, const int _index1, const int _index2, const Eigen::Vector3f &_ballCenter, const double _ballRadius);
	MeshedTriangle(const MeshedTriangle &_other);
	~MeshedTriangle();

	MeshedTriangle &operator=(const MeshedTriangle &_other);

	inline PointData getVertex(const int _n) const
	{
		if (_n < 3)
			return vertices[_n];
		else
			return std::make_pair<pcl::PointNormal *, int>(NULL, -1);
	}

	inline int getVertexIndex(const int _n) const
	{
		if (_n < 3)
			return vertices[_n].second;
		else
			return -1;
	}

	inline MeshedEdgePtr getMeshedEdge(const int _n) const
	{
		int index0 = _n % 3;
		int index1 = (_n + 1) % 3;
		int index2 = (_n + 2) % 3;
		return MeshedEdgePtr(new MeshedEdge(vertices[index0], vertices[index1], vertices[index2], ballCenter));
	}

	inline pcl::PointNormal getBallCenter() const
	{
		return ballCenter;
	}

	inline double getBallRadius()
	{
		return ballRadius;
	}

private:
	std::vector<PointData> vertices;
	pcl::PointNormal ballCenter;
	double ballRadius;
	Eigen::Vector3f normal;
};
