#include "student_code.h"
#include "halfEdgeMesh.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL {

/**
 * Evaluates one step of the de Casteljau's algorithm using the given points and
 * the scalar parameter t (class member).
 *
 * @param points A vector of points in 2D
 * @return A vector containing intermediate points or the final interpolated vector
 */
std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points) {
  std::vector<Vector2D> subdivided;
  for (int i = 1; i < points.size(); i++)
    subdivided.push_back(t * points[i - 1] + (1 - t) * points[i]);
  return subdivided;
}

/**
 * Evaluates one step of the de Casteljau's algorithm using the given points and
 * the scalar parameter t (function parameter).
 *
 * @param points    A vector of points in 3D
 * @param t         Scalar interpolation parameter
 * @return A vector containing intermediate points or the final interpolated vector
 */
std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points,
                                                double t) const {
  std::vector<Vector3D> subdivided;
  for (int i = 1; i < points.size(); i++)
    subdivided.push_back(t * points[i - 1] + (1 - t) * points[i]);
  return subdivided;
}

/**
 * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
 *
 * @param points    A vector of points in 3D
 * @param t         Scalar interpolation parameter
 * @return Final interpolated vector
 */
Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const {
  std::vector<Vector3D> subdivided = points;
  while (subdivided.size() > 1)
    subdivided = evaluateStep(subdivided, t);
  return subdivided[0];
}

/**
 * Evaluates the Bezier patch at parameter (u, v)
 *
 * @param u         Scalar interpolation parameter
 * @param v         Scalar interpolation parameter (along the other axis)
 * @return Final interpolated vector
 */
Vector3D BezierPatch::evaluate(double u, double v) const {
  std::vector<Vector3D> rows;
  for (int i = 0; i < controlPoints.size(); i++)
    rows.push_back(evaluate1D(controlPoints[i], v));
  return evaluate1D(rows, u);
}

double Face::area(void) const {
  Vector3D a = halfedge()->vertex()->position;
  Vector3D b = halfedge()->next()->vertex()->position;
  Vector3D c = halfedge()->next()->next()->vertex()->position;
  return cross(b - a, c - a).norm() / 2;
}

Vector3D Vertex::normal(void) const {
  Vector3D avg;
  HalfedgeCIter h = halfedge();
  do {
    avg += h->face()->normal() * h->face()->area();
    h = h->twin()->next();
  } while (h != halfedge());
  return avg.unit();
}

#define UPDATE_POINTERS(h)                                                                 \
  h->vertex()->halfedge() = h;                                                             \
  h->edge()->halfedge() = h;                                                               \
  h->face()->halfedge() = h;                                                               \
  h->twin()->twin() = h;                                                                   \
  h->twin()->edge() = h->edge();

#define UPDATE_POINTERS_3(h1, h2, h3)                                                      \
  UPDATE_POINTERS(h1);                                                                     \
  UPDATE_POINTERS(h2);                                                                     \
  UPDATE_POINTERS(h3);

EdgeIter HalfedgeMesh::flipEdge(EdgeIter e0) {
  HalfedgeIter ha = e0->halfedge(), haN = ha->next(), haNN = haN->next();
  HalfedgeIter hb = ha->twin(), hbN = hb->next(), hbNN = hbN->next();

  if (ha->isBoundary() || hb->isBoundary())
    return e0;

  // ha's triangle becomes ha -> hbNN -> haN
  ha->setNeighbors(hbNN, hb, haNN->vertex(), ha->edge(), ha->face());
  hbNN->setNeighbors(haN, hbNN->twin(), hbNN->vertex(), hbNN->edge(), ha->face());
  haN->setNeighbors(ha, haN->twin(), haN->vertex(), haN->edge(), ha->face());

  // hb's triangle becomes hb -> haNN -> hbN
  hb->setNeighbors(haNN, ha, hbNN->vertex(), hb->edge(), hb->face());
  haNN->setNeighbors(hbN, haNN->twin(), haNN->vertex(), haNN->edge(), hb->face());
  hbN->setNeighbors(hb, hbN->twin(), hbN->vertex(), hbN->edge(), hb->face());

  // Update pointers for each vertex, edge, and face
  UPDATE_POINTERS_3(ha, haN, haNN);
  UPDATE_POINTERS_3(hb, hbN, hbNN);

  return e0;
}

VertexIter HalfedgeMesh::splitEdge(EdgeIter e0) {
  HalfedgeIter ha = e0->halfedge(), haN = ha->next(), haNN = haN->next();
  HalfedgeIter hb = ha->twin(), hbN = hb->next(), hbNN = hbN->next();

  if (ha->isBoundary() && hb->isBoundary())
    return ha->vertex();
  if (ha->isBoundary())
    swap(ha, hb);

  // Average the positions of the two vertices
  VertexIter mid = newVertex();
  mid->position = (ha->vertex()->position + hb->vertex()->position) / 2;
  ha->vertex() = hb->vertex() = mid;
  ha->edge() = newEdge();

  // New triangle on ha's face
  // hc (haNN) -> hcN -> hcNN (hb)
  HalfedgeIter hc = newHalfedge(), hcN = newHalfedge(), hcNN = newHalfedge();
  hc->setNeighbors(hcN, haNN, ha->vertex(), haNN->edge(), newFace());
  hcN->setNeighbors(hcNN, haNN->twin(), haNN->vertex(), newEdge(), hc->face());
  hcNN->setNeighbors(hc, hb, hbN->vertex(), hb->edge(), hc->face());
  swap(hc->edge(), hcN->edge());

  // Update pointers for each vertex, edge, and face
  UPDATE_POINTERS_3(ha, haN, haNN);
  UPDATE_POINTERS_3(hc, hcN, hcNN);
  haNN->edge()->isNew = true;

  if (hb->isBoundary()) {
    // Boundary. Only one new edge to twin ha
    HalfedgeIter hdNN = newHalfedge();
    hdNN->setNeighbors(hb, ha, haN->vertex(), ha->edge(), hb->face());
    ha->twin() = hdNN;

    // Update previous halfedge's next pointer
    HalfedgeIter h = hb;
    while (h->next() != hb)
      h = h->next();
    h->next() = hdNN;

    UPDATE_POINTERS(hdNN)
    return ha->vertex();
  }

  // New triangle on hb's face
  // hd (hbNN) -> hdN -> hdNN (ha)
  HalfedgeIter hd = newHalfedge(), hdN = newHalfedge(), hdNN = newHalfedge();
  hd->setNeighbors(hdN, hbNN, hb->vertex(), hbNN->edge(), newFace());
  hdN->setNeighbors(hdNN, hbNN->twin(), hbNN->vertex(), newEdge(), hd->face());
  hdNN->setNeighbors(hd, ha, haN->vertex(), ha->edge(), hd->face());
  swap(hd->edge(), hdN->edge());

  // Update pointers for each vertex, edge, and face
  UPDATE_POINTERS_3(hb, hbN, hbNN);
  UPDATE_POINTERS_3(hd, hdN, hdNN);
  hbNN->edge()->isNew = true;

  return ha->vertex();
}

void MeshResampler::upsample(HalfedgeMesh &mesh) {
  // TODO Part 6.
  // This routine should increase the number of triangles in the mesh using Loop
  // subdivision. One possible solution is to break up the method as listed below.

  // 1. Compute new positions for all the vertices in the input mesh, using the Loop
  // subdivision rule, and store them in Vertex::newPosition. At this point, we also
  // want to mark each vertex as being a vertex of the original mesh.

  // update old vertices
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
    if (v->isBoundary()) {
      v->newPosition = v->position;
    } else {
      Vector3D neighborSum(0, 0, 0);
      int n = 0;
      HalfedgeCIter h = v->halfedge();
      do {
        n++;
        neighborSum += h->twin()->vertex()->position;
        h = h->twin()->next();
      } while (h != v->halfedge());

      double u = (n == 3) ? 3.0 / 16.0 : 3.0 / (8 * n);
      v->newPosition = (1 - n * u) * v->position + u * neighborSum;
      v->isNew = false;
    }
  }

  // 2. Compute the updated vertex positions associated with edges, and store it in
  // Edge::newPosition.

  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
    if (e->isBoundary()) {
      HalfedgeIter h = e->halfedge();
      Vector3D pos1 = h->vertex()->position;
      Vector3D pos2 = h->twin()->vertex()->position;
      Vector3D midpoint = (pos1 + pos2) * 0.5;
      e->newPosition = midpoint;
    } else {
      HalfedgeIter h = e->halfedge();
      Vector3D A = h->vertex()->position;
      Vector3D B = h->twin()->vertex()->position;
      Vector3D C = h->next()->next()->vertex()->position;
      Vector3D D = h->twin()->next()->next()->vertex()->position;
      e->newPosition = 3.0 / 8.0 * (A + B) + 1.0 / 8.0 * (C + D);
    }
  }

  // 3. Split every edge in the mesh, in any order. For future reference, we're also
  // going to store some information about which subdivide edges come from splitting an
  // edge in the original mesh, and which edges are new, by setting the flat
  // Edge::isNew. Note that in this loop, we only want to iterate over edges of the
  // original mesh---otherwise, we'll end up splitting edges that we just split (and the
  // loop will never end!)

  std::vector<EdgeIter> originalEdges;
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
    originalEdges.push_back(e);
    e->isNew = false;
  }

  for (EdgeIter e : originalEdges) {
    VertexIter newV = mesh.splitEdge(e);
    newV->newPosition = e->newPosition;
    newV->isNew = true;
  }

  // 4. Flip any new edge that connects an old and new vertex.
  for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); ++e) {
    if (e->isNew) {
      HalfedgeIter h = e->halfedge();
      bool v0isNew = h->vertex()->isNew;
      bool v1isNew = h->twin()->vertex()->isNew;
      if (v0isNew != v1isNew) {
        mesh.flipEdge(e);
      }
    }
  }

  // 5. Copy the new vertex positions into final Vertex::position.
  for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); ++v) {
    v->position = v->newPosition;
  }
}
} // namespace CGL
