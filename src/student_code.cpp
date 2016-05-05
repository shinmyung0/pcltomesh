/*
 * Student solution for UC Berkeley Project 2
 *
 * Implemented by Shin Yoon on ____.
 *
 */

#include "assert.h"
#include "student_code.h"
#include "mutablePriorityQueue.h"

namespace CGL {

    void BezierPatch::preprocess() {
        // TODO Part 1.
        // TODO If you use the matrix form for Bezier patch evaluation, you will need to
        // TODO compute your matrices based on the 16 control points here. 
        // TODO You will also need to define your matrices
        // TODO as member variables in the "BezierPatch" class.
        // TODO If you use De Casteljau's recursive algorithm, you will not need to do anything here.
        double x_coords[16];
        double y_coords[16];
        double z_coords[16];

        for (int r=0; r < 4; r++) {
          for (int c=0; c < 4; c++) {
            x_coords[c + 4*r] = controlPoints[r][c].x;
            y_coords[c + 4*r] = controlPoints[r][c].y;
            z_coords[c + 4*r] = controlPoints[r][c].z;
          }
        }
        Gx = Matrix4x4(x_coords);
        Gy = Matrix4x4(y_coords);
        Gz = Matrix4x4(z_coords);

        double basis[] = {-1.0f,  3.0f, -3.0f, 1.0f, 
                         3.0f, -6.0f,  3.0f, 0.0f,
                        -3.0f,  3.0f,  0.0f, 0.0f,
                         1.0f,  0.0f,  0.0f, 0.0f};

        basisMatrix = Matrix4x4(basis);


    }


    Vector3D BezierPatch::evaluate(double u, double v) const {
        // TODO Part 1.
        // TODO Returns the 3D point whose parametric coordinates are (u, v) on the Bezier patch.
        // TODO Note that both u and v are within [0, 1].
        Vector4D U = Vector4D(pow(u, 3), pow(u, 2), u, 1);
        Vector4D V = Vector4D(pow(v, 3), pow(v, 2), v, 1);

        Matrix4x4 Mx = basisMatrix * Gx * basisMatrix.T();
        Vector4D xv = Mx * V;

        Matrix4x4 My = basisMatrix * Gy * basisMatrix.T();
        Vector4D yv = My * V;

        Matrix4x4 Mz = basisMatrix * Gz * basisMatrix.T();
        Vector4D zv = Mz * V;

        // double x = U.x * xv.x + U.y * xv.y + U.z + xv.z + U.w * xv.w;
        // double y = U.x * yv.x + U.y * yv.y + U.z + yv.z + U.w * yv.w;
        // double z = U.x * zv.x + U.y * zv.y + U.z + zv.z + U.w * zv.w;

        double x = dot(U, xv);
        double y = dot(U, yv);
        double z = dot(U, zv);
        return Vector3D(x, y, z);
    }

    void BezierPatch::add2mesh(Polymesh* mesh) const {
        // TODO Part 1.
        // TODO Tessellate the given Bezier patch into triangles uniformly on a 8x8 grid(8x8x2=128 triangles) in parameter space.
        // TODO You will call your own evaluate function here to compute vertex positions of the tessellated triangles.
        // TODO The "addTriangle" function inherited from the "BezierPatchLoader" class may help you add triangles to the output mesh. 
        float gridDim = 8.0f;
        float stepSize = 1.0f/gridDim;

        Vector3D v0, v1, v2, v3;
        for (float u = 0; u < 1; u += stepSize) {
          for (float v = 0; v < 1; v += stepSize) {
            // add two triangles from 4 points in grid
            v0 = evaluate(u, v);
            v1 = evaluate(u + stepSize, v);
            v2 = evaluate(u, v + stepSize);
            v3 = evaluate(u + stepSize, v + stepSize);

            addTriangle(mesh, v0, v1, v3);
            addTriangle(mesh, v3, v2, v0);
            
          }
        }

    }

    Vector3D Vertex::normal(void) const
    // TODO Part 2.
    // TODO Returns an approximate unit normal at this vertex, computed by
    // TODO taking the area-weighted average of the normals of neighboring
    // TODO triangles, then normalizing.
    {
        // TODO Compute and return the area-weighted unit normal.
        
        Vector3D n(0, 0, 0);
        HalfedgeCIter h = halfedge();
        h = h->twin();
        HalfedgeCIter h_orig = h;
        Vector3D v0 = position;
        Vector3D v1, v2;
        VertexCIter Vert1, Vert2;
        do {
          Vert1 = h->vertex();
          h = h->next();
          h = h->twin();
          Vert2 = h->vertex();

          v1 = Vert1->position;
          v2 = Vert2->position;
          n += cross(v1 - v0, v2 - v0);
          
        } while (h != h_orig);

        return n.unit();
    }

    EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
        // TODO Part 3.
        // TODO This method should flip the given edge and return an iterator to the flipped edge.

        HalfedgeIter h0, h1, h2, ht0, ht1, ht2;
        h0 = e0->halfedge();
        h1 = h0->next();
        h2 = h1->next();
        ht0 = h0->twin();
        ht1 = ht0->next();
        ht2 = ht1->next();

        FaceIter f1, f2; 
        f1 = h0->face();
        f2 = ht0->face();
        
        VertexIter a, b, c, d;
        a = h2->vertex();
        b = h0->vertex();
        c = h1->vertex();
        d = ht2->vertex();

        if (e0->isBoundary() || f1->isBoundary() || f2->isBoundary()) {
          return e0;
        }

        h0->setNeighbors(ht2, ht0, a, e0, f1); 
        h1->setNeighbors(h0, h1->twin(), c, h1->edge(), f1);
        h2->setNeighbors(ht1, h2->twin(), a, h2->edge(), f2);
        ht0->setNeighbors(h2, h0, d, e0, f2); 
        ht1->setNeighbors(ht0, ht1->twin(), b, ht1->edge(), f2); 
        ht2->setNeighbors(h1, ht2->twin(), d, ht2->edge(), f1); 

        a->halfedge() = h0;
        b->halfedge() = ht1;
        c->halfedge() = h1;
        d->halfedge() = ht0;

        f1->halfedge() = h0;
        f2->halfedge() = ht0;

        return e0;
    }

    VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
        // TODO Part 4.
        // TODO This method should split the given edge and return an iterator to the newly inserted vertex.
        // TODO The halfedge of this vertex should point along the edge that was split, rather than the new edges.

        HalfedgeIter h0, h1, h2, ht0, ht1, ht2;
        h0 = e0->halfedge();
        h1 = h0->next();
        h2 = h1->next();
        ht0 = h0->twin();
        ht1 = ht0->next();
        ht2 = ht1->next();

        FaceIter f1, f2; 
        f1 = h0->face();
        f2 = ht0->face();
        
        VertexIter a, b, c, d;
        a = h2->vertex();
        b = h0->vertex();
        c = h1->vertex();
        d = ht2->vertex();

        // create midpoint vertex
        VertexIter m = newVertex();
        m->position = (b->position + c->position) / 2.0f;

        if (e0->isBoundary() || f1->isBoundary() || f2->isBoundary()) {
          return VertexIter();
        }
        

        // create 4 new h.e 4 faces, 4 edges
        HalfedgeIter nh0, nh1, nh2, nh3;
        HalfedgeIter nht0, nht1, nht2, nht3;
        FaceIter nf1, nf2, nf3, nf4;
        EdgeIter ne1, ne2, ne3, ne4;

        nh0 = newHalfedge();
        nh1 = newHalfedge();
        nh2 = newHalfedge();
        nh3 = newHalfedge();

        nht0 = newHalfedge();
        nht1 = newHalfedge();
        nht2 = newHalfedge();
        nht3 = newHalfedge();

        nf1 = newFace();
        nf2 = newFace();
        nf3 = newFace();
        nf4 = newFace();

        // ne1 = newEdge();
        ne1 = e0;
        ne2 = newEdge();
        ne3 = newEdge();
        ne4 = newEdge();

        // set new neighbors for all h.e, edges, faces, vertices
        // faces
        nf1->halfedge() = nh0;
        nf2->halfedge() = nht0;
        nf3->halfedge() = nh3;
        nf4->halfedge() = nht3;
        // edges
        ne1->halfedge() = nh0;
        ne2->halfedge() = nh2;
        ne3->halfedge() = nht1;
        ne4->halfedge() = nht3;
        // old h.e
        h1->setNeighbors(nh1, h1->twin(), c, h1->edge(), nf1);
        h2->setNeighbors(nh3, h2->twin(), a, h2->edge(), nf3);
        ht1->setNeighbors(nht2, ht1->twin(), b, ht1->edge(), nf4);
        ht2->setNeighbors(nht0, ht2->twin(), d, ht2->edge(), nf2);

        // new h.e
        nh0->setNeighbors(h1, nht0, m, ne1, nf1);
        nh1->setNeighbors(nh0, nh2, a, ne2, nf1);
        nh2->setNeighbors(h2, nh1, m, ne2, nf3);
        nh3->setNeighbors(nh2, nht3, b, ne4, nf3);

        nht0->setNeighbors(nht1, nh0, c, ne1, nf2);
        nht1->setNeighbors(ht2, nht2, m, ne3, nf2);
        nht2->setNeighbors(nht3, nht1, d, ne3, nf4);
        nht3->setNeighbors(ht1, nh3, m, ne4, nf4);

        // vertices
        a->halfedge() = nh1;
        b->halfedge() = nh3;
        c->halfedge() = nht0;
        d->halfedge() = nht2;
        m->halfedge() = nh0;
        
        // delete 2 inner h.e and 2 faces
        // deleteEdge(e0);
        deleteHalfedge(h0);
        deleteHalfedge(ht0);
        deleteFace(f1);
        deleteFace(f2);

        return m;
    }

    Vector3D computeNewVertexPosition(VertexIter v) {
      HalfedgeCIter h = v->halfedge();
      Vector3D n_sum = Vector3D(0, 0, 0);
      do {
        HalfedgeCIter ht = h->twin();
        Vector3D nv = ht->vertex()->position;
        n_sum += nv;
        h = ht->next();
      } while (h != v->halfedge());
      
      float u, n;
      n = (float) v->degree();
      if (n == 3.0f) {
        u = 3.0f / 16.0f;
      } else {
        u = 3.0f / (8.0f * n);
      }
      Vector3D original_position = v->position;
      return (1 - n*u) * original_position + u * n_sum;
    }

    Vector3D computeEdgeMidpoint(EdgeIter e) {
      HalfedgeIter h = e->halfedge();
      Vector3D A, B, C, D;
      A = h->twin()->vertex()->position;
      B = h->vertex()->position;
      C = h->next()->next()->vertex()->position;
      D = h->twin()->next()->next()->vertex()->position;
      return (3.0f / 8.0f) * (A + B) + (1.0f / 8.0f) * (C + D);
    }
      


    void MeshResampler::upsample(HalfedgeMesh& mesh)
    // TODO Part 5.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    {
        // Each vertex and edge of the original surface can be associated with a vertex in the new (subdivided) surface.
        // Therefore, our strategy for computing the subdivided vertex locations is to *first* compute the new positions
        // using the connectity of the original (coarse) mesh; navigating this mesh will be much easier than navigating
        // the new subdivided (fine) mesh, which has more elements to traverse.  We will then assign vertex positions in
        // the new mesh based on the values we computed for the original mesh.


        // TODO Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
        // TODO and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
        // TODO a vertex of the original mesh.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->isNew = false;
          v->newPosition = computeNewVertexPosition(v);
        }

        // TODO Next, compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          e->isNew = false;
          e->newPosition = computeEdgeMidpoint(e);
        }


        // TODO Next, we're going to split every edge in the mesh, in any order.  For future
        // TODO reference, we're also going to store some information about which subdivided
        // TODO edges come from splitting an edge in the original mesh, and which edges are new,
        // TODO by setting the flat Edge::isNew.  Note that in this loop, we only want to iterate
        // TODO over edges of the original mesh---otherwise, we'll end up splitting edges that we
        // TODO just split (and the loop will never end!)
        VertexIter midpoint;
        HalfedgeIter h;
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          h = e->halfedge();
          // if the edge or any of its vertices are new then old edge
          if (!e->isNew && !h->vertex()->isNew && !h->twin()->vertex()->isNew) {
            midpoint = mesh.splitEdge(e);
            h = midpoint->halfedge();

            midpoint->isNew = true;
            midpoint->newPosition = e->newPosition;
            h->edge()->isNew = false;
            h->next()->next()->edge()->isNew = true;
            h->twin()->next()->edge()->isNew = true;
            h->twin()->next()->twin()->next()->edge()->isNew = false;
          }
        }

        // TODO Now flip any new edge that connects an old and new vertex.
        for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          h = e->halfedge();
          bool old_and_new = (h->vertex()->isNew != h->twin()->vertex()->isNew) ? true : false;
          if (e->isNew && old_and_new) {
            EdgeIter flipped = mesh.flipEdge(e);
          }
        }

        // TODO Finally, copy the new vertex positions into final Vertex::position.
        for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->position = v->newPosition;
        }

    }

    // TODO Part 6.
    // TODO There's also some code you'll need to complete in "Shader/frag" file.

}
