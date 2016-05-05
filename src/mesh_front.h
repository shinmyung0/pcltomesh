#include "shared_triangle.h"
#include "ball_pivoter.h"
#include <map>
#include <list>

class MeshFront
{
public:
	MeshFront();
	~MeshFront();

	MeshedEdgePtr getActiveMeshedEdge();
	void addMeshedEdges(const SharedTrianglePtr &_triangle);
	void joinAndFix(const std::pair<int, SharedTrianglePtr> &_data, BallPivoter &_pivoter);
	void setInactive(MeshedEdgePtr &_edge);

	inline bool inFront(const int _index)
	{
		return points.find(_index) != points.end();
	}

private:
	std::list<MeshedEdgePtr>::iterator isPresent(const MeshedEdgePtr &_edge);
	void addMeshedEdgePoints(std::list<MeshedEdgePtr>::iterator &_edge);
	void removeMeshedEdgePoints(MeshedEdgePtr &_edge);

	std::list<MeshedEdgePtr> front;
	std::list<MeshedEdgePtr>::iterator pos;
	std::map<int, std::map<MeshedEdgePtr, std::list<MeshedEdgePtr>::iterator> > points;
};
