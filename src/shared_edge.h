#pragma once

#include <pcl/common/common.h>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <ostream>

// MeshedEdge smart pointer
class MeshedEdge;
typedef boost::shared_ptr<MeshedEdge> MeshedEdgePtr;

typedef std::pair<pcl::PointNormal *, int> PointData;

class MeshedEdge
{
public:
	MeshedEdge();
	MeshedEdge(const PointData &_v0, const PointData &_v1, const PointData &_opposite, const pcl::PointNormal &_center);
	MeshedEdge(const MeshedEdge &_other);
	~MeshedEdge();

	friend std::ostream &operator<<(std::ostream &_stream, const MeshedEdge &_edge);

	MeshedEdge &operator=(const MeshedEdge &_other);
	bool operator<(const MeshedEdge &_other) const;
	bool operator==(const MeshedEdge &_other) const;
	bool operator!=(const MeshedEdge &_other) const;

	inline void setActive(const bool _active)
	{
		active = _active;
	}

	inline bool isActive() const
	{
		return active;
	}

	inline PointData getVertex(const int _n) const
	{
		if (_n < 2)
			return vertices[_n];
		else
			return std::make_pair<pcl::PointNormal *, int>(NULL, -1);
	}

	inline PointData getOppositeVertex() const
	{
		return oppositeVertex;
	}

	inline pcl::PointNormal getMiddlePoint() const
	{
		return middlePoint;
	}

	inline pcl::PointNormal getBallCenter() const
	{
		return ballCenter;
	}

	inline double getPivotingRadius() const
	{
		return pivotingRadius;
	}

	inline std::string toString() const
	{
		return dynamic_cast<std::stringstream &>((std::stringstream() << std::dec << vertices[0].second << "-" << vertices[1].second)).str();
	}

private:
	inline void setId()
	{
		static long currentId = 0;
		id = currentId++;
	}

	std::vector<PointData> vertices;
	PointData oppositeVertex;
	pcl::PointNormal ballCenter;
	pcl::PointNormal middlePoint;
	double pivotingRadius;
	bool active;

	long id;
	std::string str;
};
