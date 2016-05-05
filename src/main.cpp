#include "CGL/CGL.h"

#include "shared_edge.h"
#include "util.h"
#include "shared_triangle.h"
#include "mesh_front.h"
#include "ball_pivoter.h"
#include "collada.h"
#include "meshEdit.h"
#include "bezierPatch.h"
#include "mergeVertices.h"
#include "shaderUtils.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>



#include <iostream>

using namespace std;
using namespace CGL;
using namespace pcl;

#define msg(s) cerr << "[Collada Viewer] " << s << endl;

int loadFile(MeshEdit* collada_viewer, const char* path, double ball_radius) {

    printf("Loading file... %s\n", path);
    Scene* scene = new Scene();
    string path_str = path;

    if (ball_radius == 0) {
      //run the visualizer
      PointCloud<PointXYZ>::Ptr cloud (new PointCloud<PointXYZ>);
      int result = io::loadPCDFile(path, *cloud);
      if (result < 0) {
        PCL_ERROR("Coudln't read the file!!");
        return -1;
      }

      cout << "Running visualizer ... \n";
      pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
      viewer.showCloud (cloud);
      while (!viewer.wasStopped ()) {

      }
      
    }

    // Parse ply data
    else if (path_str.substr(path_str.length() - 4, 4) == ".pcd") {

      // initialize the camera
      Camera* cam = new Camera();
      cam->type = CAMERA;
      Node node;
      node.instance = cam;
      scene->nodes.push_back(node);


      double initial_radius = ball_radius;


      PointCloud<PointNormal>::Ptr cloud(new PointCloud<PointNormal>());
      if (!Util::getCloudAndNormals(path, cloud))
      {
        cout << "ERROR: loading failed\n";
        return -1;
      }
      cout << "Loaded " << cloud->size() << " points in cloud\n";

      
      BallPivoter pivoter(cloud, ball_radius);
      MeshFront front;
      vector<SharedTrianglePtr> mesh;

      cout << "Building mesh with ball r=" << ball_radius << "\n";
      while (true)
      {
        // Pivot from the current front
        MeshedEdgePtr edge;
        while ((edge = front.getActiveMeshedEdge()) != NULL)
        {
          // reset to initial radius
          ball_radius = initial_radius;
          bool found = false;
          //cout << "Testing edge " << *edge << "\n";

          pair<int, SharedTrianglePtr> data = pivoter.pivot(edge);
          

          // Increase the radius or until it hits a threshold
          while (ball_radius <= initial_radius * 10.0f) {
            if (data.first != -1 && (!pivoter.isUsed(data.first) || front.inFront(data.first))) {

              mesh.push_back(data.second);
              front.joinAndFix(data, pivoter);
              found = true;

              break;

            }
  
            // increase the radius
            ball_radius += initial_radius * 0.5f;
            cout << "Ball radius increased to : " << ball_radius << endl;


          }
          if (!found) {
            front.setInactive(edge);


          }
        }


        // If there are no more active edges in the front, find a new seed
        SharedTrianglePtr seed;
        cout << "Searching a seed\n";
        if ((seed = pivoter.findSeed()) != NULL)
        {
          mesh.push_back(seed);
          front.addMeshedEdges(seed);

        }
        else
          break;
      }


      // Extract into a mesh
      Polymesh* sceneMesh = new Polymesh();
      sceneMesh->type = POLYMESH;


      vector<vector<int> > sides;
      vector<PointNormal *> pointArray;
      map<PointNormal *, int> pointMap;
      int numFaces = mesh.size();

      // Organize the points and faces
      for (int k=0; k < numFaces; k++) {
        
        SharedTrianglePtr t = mesh[k];
        sides.push_back(vector<int>());

        for (int i = 0; i < 3; i++)
        {
          PointNormal *p = t->getVertex(i).first;
          
          if (pointMap.find(p) == pointMap.end())
          {
            pointMap[p] = pointArray.size();
            pointArray.push_back(p);
          }

          sides.back().push_back(pointMap[p]);
        }
      }

      // extract to the sceneMesh
      int points = pointMap.size();

      // extract points onto the sceneMesh
      for (size_t i=0; i < pointArray.size(); i++) {
        Vector3D v = Vector3D(pointArray[i]->x, pointArray[i]->y, pointArray[i]->z);
        sceneMesh->vertices.push_back(v);
      }

      // extract as polygons
      for (size_t k=0; k < sides.size(); k++) {
        vector<int> polyIdx = sides[k];
        Polygon face;
        face.vertex_indices.push_back(polyIdx[0]);
        face.vertex_indices.push_back(polyIdx[1]);
        face.vertex_indices.push_back(polyIdx[2]);
        sceneMesh->polygons.push_back(face);
      }


      // after we are done adding everything 
      // add the sceneMesh to the scene 
      node.instance = sceneMesh;
      scene->nodes.push_back(node);

    } else if (path_str.substr(path_str.length() - 4, 4) == ".dae") {
        if (ColladaParser::load(path, scene) < 0) {
            delete scene;
            return -1;
        }
    } else if (path_str.substr(path_str.length() - 4, 4) == ".bez") {
        Camera* cam = new Camera();
        cam->type = CAMERA;
        Node node;
        node.instance = cam;
        scene->nodes.push_back(node);
        Polymesh* mesh = new Polymesh();

        FILE* file = fopen(path, "r");
        int n = 0;
        fscanf(file, "%d", &n);
        for (int i = 0; i < n; i++) {
            BezierPatch patch;
            patch.loadControlPoints(file);
            patch.add2mesh(mesh);
            mergeVertices(mesh);
        }
        fclose(file);

        mesh->type = POLYMESH;
        node.instance = mesh;
        scene->nodes.push_back(node);
    } else {
        return -1;
    }

    collada_viewer->load(scene);

    GLuint tex = makeTex("envmap/envmap.png");
    if (!tex) tex = makeTex("../envmap/envmap.png");
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, tex);
    glActiveTexture(GL_TEXTURE2);

    return 0;
}

int main(int argc, char** argv) {

    double radius;
    size_t pos;
    radius = stod(argv[2], &pos);

    // create collada_viewer
    MeshEdit* collada_viewer = new MeshEdit();
    Viewer viewer;

    if (radius != 0) {
      // create viewer
      viewer = Viewer();
      // set collada_viewer as renderer
      viewer.set_renderer(collada_viewer);

      // init viewer
      viewer.init();

    }
    
 
   // load tests
    
    if (argc == 3) {

        
        if (loadFile(collada_viewer, argv[1], radius) < 0) exit(0);
    } else {
        msg("Usage: collada_viewer <path to scene file> <ball radius>");
        exit(0);
    }

    // start viewer
    if (radius != 0) {
      viewer.start();
    }

    return 0;
}
