
#include "shared_edge.h"
#include <eigen3/Eigen/src/Core/Matrix.h>
#include "util.h"

std::ostream &operator<<(std::ostream &_stream, const MeshedEdge &_edge)
{
	_stream << "(" << _edge.vertices[0].second << ", " << _edge.vertices[1].second << ")";
	return _stream;
}

MeshedEdge::MeshedEdge()
{
	vertices.clear();
	oppositeVertex = std::make_pair<pcl::PointNormal *, int>(NULL, -1);
	ballCenter = middlePoint = pcl::PointNormal();
	pivotingRadius = 0;
	active = false;
	setId();
	str = "";
}

MeshedEdge::MeshedEdge(const PointData &_v0, const PointData &_v1, const PointData &_opposite, const pcl::PointNormal &_ballCenter)
{
	vertices.push_back(_v0);
	vertices.push_back(_v1);
	oppositeVertex = _opposite;

	ballCenter = _ballCenter;
	middlePoint = Util::makePointNormal((_v0.first->x + _v1.first->x) * 0.5, (_v0.first->y + _v1.first->y) * 0.5, (_v0.first->z + _v1.first->z) * 0.5);

	Eigen::Vector3f m = middlePoint.getVector3fMap();
	Eigen::Vector3f c = ballCenter.getVector3fMap();
	pivotingRadius = (m - c).norm();

	active = true;
	setId();

	str = this->toString();
}

MeshedEdge::MeshedEdge(const MeshedEdge &_other)
{
	vertices = _other.vertices;
	oppositeVertex = _other.oppositeVertex;

	ballCenter = _other.ballCenter;
	pivotingRadius = _other.pivotingRadius;

	middlePoint = _other.middlePoint;
	active = _other.active;
	id = _other.id;

	str = _other.str;
}

MeshedEdge::~MeshedEdge()
{
}

MeshedEdge &MeshedEdge::operator=(const MeshedEdge &_other)
{
	if (this != &_other)
	{
		vertices = _other.vertices;
		oppositeVertex = _other.oppositeVertex;

		ballCenter = _other.ballCenter;
		pivotingRadius = _other.pivotingRadius;

		middlePoint = _other.middlePoint;
		active = _other.active;
		id = _other.id;

		str = _other.str;
	}

	return *this;
}

bool MeshedEdge::operator<(const MeshedEdge &_other) const
{
	return pivotingRadius < _other.pivotingRadius;
}

bool MeshedEdge::operator==(const MeshedEdge &_other) const
{
	bool equals = (vertices[0] == _other.vertices[0] || vertices[0] == _other.vertices[1]) && (vertices[1] == _other.vertices[0] || vertices[1] == _other.vertices[1]);

	return equals;
}

bool MeshedEdge::operator!=(const MeshedEdge &_other) const
{
	return !this->operator==(_other);
}
