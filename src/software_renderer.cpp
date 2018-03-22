#include "software_renderer.h"

#include <cmath>
#include <vector>
#include <iostream>
#include <algorithm>
#include <assert.h>

#include "triangulation.h"

using namespace std;

namespace CMU462 {

// Implements SoftwareRenderer //

//if transform matrix is changed, reference render breaks.
//all right, I'll make a copy and modify it.
static void shift(SVGElement *element, Matrix3x3 matrix3x3, SVGElement *ref) {
  if (element == nullptr) {
    return;
  }
//  assert(element->type == ref->type);
//  assert(element->transform == ref->transform);
  if (element->type == GROUP) {
    Group *g = dynamic_cast<Group *>(element);
    assert(g != nullptr);
    matrix3x3 = matrix3x3 * g->transform;
    for (int i = 0; i < g->elements.size(); i++) {
      shift(g->elements[i], matrix3x3, ((Group *) ref)->elements[i]);
    }
    return;
  }
  element->transform = matrix3x3 * element->transform;
  //transform to canvas coordinates
  //every method in draw_XXXX performed transformation from canvas to screen
  element->applyTransformation();
}

void SoftwareRendererImp::draw_svg(const SVG &originalSVG) {

  // set top level transformation
  transformation = canvas_to_screen;

  // do transformation
  // instrument code consider the param unchanged across calls as it pass the same param to this and ref render
  // so I make a copy here.
  SVG svg = originalSVG;
  for (int i = 0; i < svg.elements.size(); i++) {
    shift(svg.elements[i], Matrix3x3::identity(), originalSVG.elements[i]);
  }


  // draw all elements
  for (size_t i = 0; i < svg.elements.size(); ++i) {
    draw_element(svg.elements[i]);
  }

  // draw canvas outline
  Vector2D a = transform(Vector2D(0, 0));
  a.x--;
  a.y--;
  Vector2D b = transform(Vector2D(svg.width, 0));
  b.x++;
  b.y--;
  Vector2D c = transform(Vector2D(0, svg.height));
  c.x--;
  c.y++;
  Vector2D d = transform(Vector2D(svg.width, svg.height));
  d.x++;
  d.y++;

  rasterize_line(a.x, a.y, b.x, b.y, Color::Black);
  rasterize_line(a.x, a.y, c.x, c.y, Color::Black);
  rasterize_line(d.x, d.y, b.x, b.y, Color::Black);
  rasterize_line(d.x, d.y, c.x, c.y, Color::Black);

  // resolve and send to render target
  resolve();

}

void SoftwareRendererImp::set_sample_rate(size_t sample_rate) {

  // Task 4: 
  // You may want to modify this for supersampling support
  if (sample_rate == this->sample_rate) {
    return;
  }
  this->sample_rate = sample_rate;
//  double v = sample_rate * 1.0 / this->sample_rate;
//  vector<double> tmp(9);
//  tmp[0] = v;
//  tmp[4] = v;
//  tmp[8] = 1;
//  Matrix3x3 m(tmp.data());
//  this->transformation = m * this->transformation;
  reset_buffer();
}

void SoftwareRendererImp::set_render_target(unsigned char *render_target,
                                            size_t width, size_t height) {

  // Task 4: 
  // You may want to modify this for supersampling support
  this->render_target = render_target;
  this->target_w = width;
  this->target_h = height;

  reset_buffer();
}

void SoftwareRendererImp::reset_buffer() {
  if (sample_rate == 1) {
    this->tmp_render_target = this->render_target;
    this->tmp_target_h = this->target_h;
    this->tmp_target_w = this->target_w;
  } else {
    this->tmp_target_w = sample_rate * this->target_w;
    this->tmp_target_h = sample_rate * this->target_h;
    if (this->tmp_render_target != this->render_target) {
      delete (this->tmp_render_target);
    }
    this->tmp_render_target = new unsigned char[4 * this->tmp_target_h * this->tmp_target_w];
    for (int i = 0; i < 4 * this->tmp_target_h * this->tmp_target_w; i++) {
      this->tmp_render_target[i] = 0;
    }
  }
}

void SoftwareRendererImp::draw_element(SVGElement *element) {

  // Task 5 (part 1):
  // Modify this to implement the transformation stack

  switch (element->type) {
  case POINT:draw_point(static_cast<Point &>(*element));
    break;
  case LINE:draw_line(static_cast<Line &>(*element));
    break;
  case POLYLINE:draw_polyline(static_cast<Polyline &>(*element));
    break;
  case RECT:draw_rect(static_cast<Rect &>(*element));
    break;
  case POLYGON:draw_polygon(static_cast<Polygon &>(*element));
    break;
  case ELLIPSE:draw_ellipse(static_cast<Ellipse &>(*element));
    break;
  case IMAGE:draw_image(static_cast<Image &>(*element));
    break;
  case GROUP:draw_group(static_cast<Group &>(*element));
    break;
  default:break;
  }

}


// Primitive Drawing //

void SoftwareRendererImp::draw_point(Point &point) {

  Vector2D p = transform(point.position);
  rasterize_point(p.x, p.y, point.style.fillColor);

}

void SoftwareRendererImp::draw_line(Line &line) {

  Vector2D p0 = transform(line.from);
  Vector2D p1 = transform(line.to);
  rasterize_line(p0.x, p0.y, p1.x, p1.y, line.style.strokeColor);

}

void SoftwareRendererImp::draw_polyline(Polyline &polyline) {

  Color c = polyline.style.strokeColor;

  if (c.a != 0) {
    int nPoints = polyline.points.size();
    for (int i = 0; i < nPoints - 1; i++) {
      Vector2D p0 = transform(polyline.points[(i + 0) % nPoints]);
      Vector2D p1 = transform(polyline.points[(i + 1) % nPoints]);
      rasterize_line(p0.x, p0.y, p1.x, p1.y, c);
    }
  }
}

void SoftwareRendererImp::draw_rect(Rect &rect) {

  Color c;

  // draw as two triangles
  float x = rect.position.x;
  float y = rect.position.y;
  float w = rect.dimension.x;
  float h = rect.dimension.y;

  Vector2D p0 = transform(Vector2D(x, y));
  Vector2D p1 = transform(Vector2D(x + w, y));
  Vector2D p2 = transform(Vector2D(x, y + h));
  Vector2D p3 = transform(Vector2D(x + w, y + h));

  // draw fill
  c = rect.style.fillColor;
  if (c.a != 0) {
    rasterize_triangle(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c);
    rasterize_triangle(p2.x, p2.y, p1.x, p1.y, p3.x, p3.y, c);
  }

  // draw outline
  c = rect.style.strokeColor;
  if (c.a != 0) {
    rasterize_line(p0.x, p0.y, p1.x, p1.y, c);
    rasterize_line(p1.x, p1.y, p3.x, p3.y, c);
    rasterize_line(p3.x, p3.y, p2.x, p2.y, c);
    rasterize_line(p2.x, p2.y, p0.x, p0.y, c);
  }

}

void SoftwareRendererImp::draw_polygon(Polygon &polygon) {

  Color c;

  // draw fill
  c = polygon.style.fillColor;
  if (c.a != 0) {

    // triangulate
    vector<Vector2D> triangles;
    triangulate(polygon, triangles);

    // draw as triangles
    for (size_t i = 0; i < triangles.size(); i += 3) {
      Vector2D p0 = transform(triangles[i + 0]);
      Vector2D p1 = transform(triangles[i + 1]);
      Vector2D p2 = transform(triangles[i + 2]);
      rasterize_triangle(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, c);
    }
  }

  // draw outline
  c = polygon.style.strokeColor;
  if (c.a != 0) {
    int nPoints = polygon.points.size();
    for (int i = 0; i < nPoints; i++) {
      Vector2D p0 = transform(polygon.points[(i + 0) % nPoints]);
      Vector2D p1 = transform(polygon.points[(i + 1) % nPoints]);
      rasterize_line(p0.x, p0.y, p1.x, p1.y, c);
    }
  }
}

void SoftwareRendererImp::draw_ellipse(Ellipse &ellipse) {

  // Extra credit 

}

void SoftwareRendererImp::draw_image(Image &image) {

  Vector2D p0 = transform(image.position);
  Vector2D p1 = transform(image.position + image.dimension);

  rasterize_image(p0.x, p0.y, p1.x, p1.y, image.tex);
}

void SoftwareRendererImp::draw_group(Group &group) {

  for (size_t i = 0; i < group.elements.size(); ++i) {
    draw_element(group.elements[i]);
  }

}

// Rasterization //

// The input arguments in the rasterization functions 
// below are all defined in screen space coordinates

void SoftwareRendererImp::rasterize_point(float x, float y, Color color) {

  x *= sample_rate;
  y *= sample_rate;
  // fill in the nearest pixel
  int sx = (int) floor(x);
  int sy = (int) floor(y);

  // check bounds
  if (sx < 0 || sx >= tmp_target_w)
    return;
  if (sy < 0 || sy >= tmp_target_h)
    return;

  // fill sample - NOT doing alpha blending!
  tmp_render_target[4 * (sx + sy * tmp_target_w)] = (uint8_t) (color.r * 255);
  tmp_render_target[4 * (sx + sy * tmp_target_w) + 1] = (uint8_t) (color.g * 255);
  tmp_render_target[4 * (sx + sy * tmp_target_w) + 2] = (uint8_t) (color.b * 255);
  tmp_render_target[4 * (sx + sy * tmp_target_w) + 3] = (uint8_t) (color.a * 255);

}

void SoftwareRendererImp::rasterize_line(float x0, float y0,
                                         float x1, float y1,
                                         Color color) {

  // Task 2: 
  // Implement line rasterization
  if (x0 > x1) {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }
  int dx = x1 - x0;
  int dy = y1 - y0;
  if (abs(x1 - x0) < 0.001) {
    if (y0 > y1) {
      swap(y0, y1);
    }
    for (int i = y0; i <= y1; i++) {
      rasterize_point(x0, i, color);
    }
    return;
  }
  if (dy == 0) {
    if (x0 > x1) {
      swap(x0, x1);
    }
    for (int i = x0; i <= x1; i++) {
      rasterize_point(i, y0, color);
    }
    return;
  }
  double slope = (y1 - y0) / (x1 - x0);
  if (slope > 0) {
    if (abs(slope) < 1) {
      int eps = 0;
      int y = y0;
      for (int x = x0; x <= x1; x++) {
        eps += dy;
        if (eps << 2 >= dx) {
          y++;
          eps -= dx;
        }
        rasterize_point(x, y, color);
      }
    } else {
      int eps = 0;
      int x = x0;
      for (int y = y0; y <= y1; y++) {
        eps += dx;
        if (eps << 2 >= dy) {
          x++;
          eps -= dy;
        }
        rasterize_point(x, y, color);
      }
    }
  } else {
    if (abs(slope) < 1) {
      int eps = 0;
      int y = y0;
      for (int x = x0; x <= x1; x++) {
        eps += dy;
        if (eps << 2 <= -dx) {
          eps += dx;
          y--;
        }
        rasterize_point(x, y, color);
      }
    } else {
      int eps = 0;
      int x = x0;
      for (int y = y0; y >= y1; y--) {
        eps -= dx;
        if (eps << 2 <= dy) {
          eps -= dy;
          x++;
        }
        rasterize_point(x, y, color);
      }
    }
  }
}

void SoftwareRendererImp::rasterize_triangle(float x0, float y0,
                                             float x1, float y1,
                                             float x2, float y2,
                                             Color color) {
  // Task 3: 
  // Implement triangle rasterization
  Vector2D BA(x1 - x0, y1 - y0);//v0
  Vector2D CA(x2 - x0, y2 - y0);//v1
  double dot00 = dot(BA, BA);
  double dot01 = dot(BA, CA);
  double dot11 = dot(CA, CA);

  for (int j = 0; j < tmp_target_h; j++) {
    for (int i = 0; i < tmp_target_w; i++) {
      Vector2D PA(0.5f + i - x0, 0.5f + j - y0);
      double dot02 = dot(BA, PA);
      double dot12 = dot(CA, PA);
      double b = 1.0 / (dot00 * dot11 - dot01 * dot01);
      double u = (dot11 * dot02 - dot01 * dot12) * b;
      double v = (dot00 * dot12 - dot01 * dot02) * b;
      if (u >= 0 && v >= 0 && u + v < 1) {
        rasterize_point(i, j, color);
      }
    }
  }
}

void SoftwareRendererImp::rasterize_image(float x0, float y0,
                                          float x1, float y1,
                                          Texture &tex) {
  // Task 6: 
  // Implement image rasterization

}

// resolve samples to render target
void SoftwareRendererImp::resolve(void) {

  // Task 4: 
  // Implement supersampling
  // You may also need to modify other functions marked with "Task 4".
  if (sample_rate == 1) {
    return;
  }
  for (int j = 0; j < this->target_h; j++) {
    for (int i = 0; i < this->target_w; i++) {
      int cnt = 0;
      int color = 0;
      for (int n = j * this->sample_rate; n < (j + 1) * this->sample_rate;
           n++) {
        for (int m = i * this->sample_rate; m < (i + 1) * this->sample_rate;
             m++) {

          //i don't know how to deal with multi color in one sample point for now
          //assume that there is only one kind of color
          auto idx = 4 * (n * this->tmp_target_w + m);
          cnt += this->tmp_render_target[idx + 3];
          int c = (int) this->tmp_render_target[idx];
          if (c > 0) {
            color = c;
          }
        }
      }
//      cnt /= 1.0 * sample_rate * sample_rate;
      *(int *) &this->render_target[4 * (j * this->target_w + i)] = color;
      *(int *) &this->render_target[4 * (j * this->target_w + i) + 3] = cnt & 0xff;
    }
  }
  return;

}

} // namespace CMU462
