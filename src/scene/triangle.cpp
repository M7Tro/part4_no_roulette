#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.

  //A ray is parametrized by t and only intersections with t >= 0

  //How do I check if there is an intersection between a ray and a triangle?
  //Ray is defined by an origin point and a direction vector
  //Triangle is defined by three vertices
  //If there is an intersection, then there is value of parameter t
  //for which the ray touches the plane of the triangle.

  //Use Moller Trumbore algorithm
  Vector3D E2 = p2 - p1;
  Vector3D E3 = p3 - p1;
  Vector3D S = r.o - p1;
  Vector3D S2 = cross(r.d, E3);
  Vector3D S3 = cross(S, E2);

  double t = 1.0 / dot(S2, E2) * dot(S3, E3);
  double b2 = 1.0 / dot(S2, E2) * dot(S2, S);
  double b3 = 1.0 / dot(S2, E2) * dot(S3, r.d);
  double b1 = (1.0 - b2 - b3);

  //Check if ray does not interesect triangle:
  if (b1 < 0 || b2 < 0 || b3 < 0) return false;

  //Check if t is out of bounds:
  if (t < r.min_t || t > r.max_t) return false;

  r.max_t = t;

  return true;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  //Use Moller Trumbore algorithm
  Vector3D E2 = p2 - p1;
  Vector3D E3 = p3 - p1;
  Vector3D S = r.o - p1;
  Vector3D S2 = cross(r.d, E3);
  Vector3D S3 = cross(S, E2);

  double t = 1.0 / dot(S2, E2) * dot(S3, E3);
  double b2 = 1.0 / dot(S2, E2) * dot(S2, S);
  double b3 = 1.0 / dot(S2, E2) * dot(S3, r.d);
  double b1 = (1.0 - b2 - b3);

  //Check if ray does not interesect triangle:
  if (b1 < 0 || b2 < 0 || b3 < 0) return false;

  //Check if t is out of bounds:
  if (t < r.min_t || t > r.max_t) return false;

  r.max_t = t;

  //Calculate surface normal at the intersection
  Vector3D n = b1 * n1 + b2 * n2 + b3 * n3;
  n.normalize();

  //populate the *isect:
  isect -> t = t;
  isect -> primitive = this;
  isect -> n = n;
  isect -> bsdf = get_bsdf();

  return true;
}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
