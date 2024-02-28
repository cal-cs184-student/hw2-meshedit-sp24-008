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
  h->twin()->twin() = h;

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

  if (ha->isBoundary() || hb->isBoundary())
    return ha->vertex();

  // Average the positions of the two vertices
  VertexIter mid = newVertex();
  mid->position = (ha->vertex()->position + hb->vertex()->position) / 2;
  ha->vertex() = hb->vertex() = mid;
  ha->edge() = newEdge();

  // Create new triangles
  HalfedgeIter hc = newHalfedge(), hcN = newHalfedge(), hcNN = newHalfedge();
  HalfedgeIter hd = newHalfedge(), hdN = newHalfedge(), hdNN = newHalfedge();

  // new triangle 1 is hc (haNN) -> hcN -> hcNN (hb)
  hc->face() = newFace();
  hc->setNeighbors(hcN, haNN, ha->vertex(), haNN->edge(), hc->face());
  hcN->setNeighbors(hcNN, haNN->twin(), haNN->vertex(), newEdge(), hc->face());
  hcNN->setNeighbors(hc, hb, hbN->vertex(), hb->edge(), hc->face());
  hcN->twin()->edge() = hcN->edge();

  // new triangle 2 is hd (hbNN) -> hdN -> hdNN (ha)
  hd->face() = newFace();
  hd->setNeighbors(hdN, hbNN, hb->vertex(), hbNN->edge(), hd->face());
  hdN->setNeighbors(hdNN, hbNN->twin(), hbNN->vertex(), newEdge(), hd->face());
  hdNN->setNeighbors(hd, ha, haN->vertex(), ha->edge(), hd->face());
  hdN->twin()->edge() = hdN->edge();

  // hcN takes the place of haNN and hdN takes the place of hbNN

  // Update pointers for each vertex, edge, and face
  UPDATE_POINTERS_3(ha, haN, haNN);
  UPDATE_POINTERS_3(hb, hbN, hbNN);
  UPDATE_POINTERS_3(hc, hcN, hcNN);
  UPDATE_POINTERS_3(hd, hdN, hdNN);

  return ha->vertex();
}

void MeshResampler::upsample(HalfedgeMesh &mesh) {
  // TODO Part 6.
  // This routine should increase the number of triangles in the mesh using Loop
  // subdivision. One possible solution is to break up the method as listed below.

  // 1. Compute new positions for all the vertices in the input mesh, using the Loop
  // subdivision rule, and store them in Vertex::newPosition. At this point, we also
  // want to mark each vertex as being a vertex of the original mesh.

  // 2. Compute the updated vertex positions associated with edges, and store it in
  // Edge::newPosition.

  // 3. Split every edge in the mesh, in any order. For future reference, we're also
  // going to store some information about which subdivide edges come from splitting an
  // edge in the original mesh, and which edges are new, by setting the flat
  // Edge::isNew. Note that in this loop, we only want to iterate over edges of the
  // original mesh---otherwise, we'll end up splitting edges that we just split (and the
  // loop will never end!)

  // 4. Flip any new edge that connects an old and new vertex.

  // 5. Copy the new vertex positions into final Vertex::position.
}
} // namespace CGL
