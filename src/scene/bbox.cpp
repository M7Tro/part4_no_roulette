#include "bbox.h"

#include "GL/glew.h"

#include <algorithm>
#include <iostream>

namespace CGL {

bool BBox::intersect(const Ray& r, double& t0, double& t1) const {
  double t_x1 = (min.x - r.o.x) / r.d.x;
  double t_x2 = (max.x - r.o.x) / r.d.x;
  if (t_x2 < t_x1) {
    std::swap(t_x1, t_x2);
  }

  double t_y1 = (min.y - r.o.y) / r.d.y;
  double t_y2 = (max.y - r.o.y) / r.d.y;
  if (t_y2 < t_y1) {
    std::swap(t_y1, t_y2);
  }


  double t_z1 = (min.z - r.o.z) / r.d.z;
  double t_z2 = (max.z - r.o.z) / r.d.z;
  if (t_z2 < t_z1) {
    std::swap(t_z1, t_z2);
  }

  double t_min = std::max(t_x1, std::max(t_y1, t_z1));
  double t_max = std::min(t_x2, std::min(t_y2, t_z2));

  bool within_interval = t_min <= t1 && t_max >= t0 && t_min <= t_max;
  return within_interval;
}

void BBox::draw(Color c, float alpha) const {

  glColor4f(c.r, c.g, c.b, alpha);

  // top
  glBegin(GL_LINE_STRIP);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(max.x, max.y, max.z);
  glEnd();

  // bottom
  glBegin(GL_LINE_STRIP);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, min.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glEnd();

  // side
  glBegin(GL_LINES);
  glVertex3d(max.x, max.y, max.z);
  glVertex3d(max.x, min.y, max.z);
  glVertex3d(max.x, max.y, min.z);
  glVertex3d(max.x, min.y, min.z);
  glVertex3d(min.x, max.y, min.z);
  glVertex3d(min.x, min.y, min.z);
  glVertex3d(min.x, max.y, max.z);
  glVertex3d(min.x, min.y, max.z);
  glEnd();

}

std::ostream& operator<<(std::ostream& os, const BBox& b) {
  return os << "BBOX(" << b.min << ", " << b.max << ")";
}

} // namespace CGL