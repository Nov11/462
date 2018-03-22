#include "viewport.h"

#include "CMU462.h"

namespace CMU462 {

void ViewportImp::set_viewbox(float x, float y, float span) {

  // Task 5 (part 2): 
  // Set svg to normalized device coordinate transformation. Your input
  // arguments are defined as SVG canvans coordinates.
  this->x = x;
  this->y = y;
  this->span = span;

  //first translate to original point
  //then scale to 1/(2*span)
  Matrix3x3 m1;
  m1(0, 0) = 1.0 / 2 / span;
  m1(1, 1) = 1.0 / 2 / span;
  m1(2, 2) = 1;
//  Vector3D vector3D(-x, -y, 1);
//  auto ret = m1 * vector3D;
  Matrix3x3 m2;
  m2(0, 0) = 1;
  m2(1, 1) = 1;
  m2(2, 2) = 1;
  m2(0, 2) = span - x;
  m2(1, 2) = span - y;
//  ret.z = 1;
//  auto tmp = ret;
//  auto tt = m2 * ret;;
//  ret = m2 * ret;
//  Vector3D vv(-1, -1, 1);
//  auto rvv = m2 * vv;

  set_canvas_to_norm(m1 * m2);
}

void ViewportImp::update_viewbox(float dx, float dy, float scale) {

  this->x -= dx;
  this->y -= dy;
  this->span *= scale;
  set_viewbox(x, y, span);
}

} // namespace CMU462