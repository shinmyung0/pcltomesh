#include "CGL/CLG.h";

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

#define COMPARISON_EPSILON  1e-10

using namespace pcl;
using namespace CGL;


namespace MeshGenerator {

  struct Edge {
    Vector3D start;
    Vector3D end;
    Vector3D across;
  }

  struct MeshGenerator {
    float 
