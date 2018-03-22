#ifndef CMU462_SVG_H
#define CMU462_SVG_H

#include <map>
#include <vector>

#include "color.h"
#include "texture.h"
#include "vector2D.h"
#include "matrix3x3.h"

#include "tinyxml2.h"
using namespace tinyxml2;

namespace CMU462 {

typedef enum e_SVGElementType {
  NONE = 0,
  POINT,
  LINE,
  POLYLINE,
  RECT,
  POLYGON,
  ELLIPSE,
  IMAGE,
  GROUP
} SVGElementType;

struct Style {
  Color strokeColor;
  Color fillColor;
  float strokeWidth;
  float miterLimit;
};

struct SVGElement {

  SVGElement(SVGElementType _type)
      : type(_type), transform(Matrix3x3::identity()) {}

  virtual ~SVGElement() {}

  // primitive type
  SVGElementType type;

  // styling
  Style style;

  // transformation list
  Matrix3x3 transform;

  //apply transformation
  virtual void applyTransformation() {}
  void doTransformation(Vector2D &vex) {
    Vector3D tmp(vex.x, vex.y, 1);
    tmp = transform * tmp;
    vex = Vector2D(tmp.x / tmp.z, tmp.y / tmp.z);
  }

  SVGElement(const SVGElement &svgElement)
      : type(svgElement.type), style(svgElement.style), transform(svgElement.transform) {}

  virtual SVGElement *copy() {
    return new SVGElement(*this);
  }
};

struct Group : SVGElement {

  Group() : SVGElement(GROUP) {}
  std::vector<SVGElement *> elements;

  ~Group();
  Group(const Group &group) : SVGElement(group) {
    for (auto item : group.elements) {
      elements.push_back(item->copy());
    }
  }
  Group *copy() override {
    return new Group(*this);
  }
};

struct Point : SVGElement {

  Point() : SVGElement(POINT) {}
  Vector2D position;
  void applyTransformation() override {
    doTransformation(position);
  }
  Point *copy() override {
    return new Point(*this);
  }
};

struct Line : SVGElement {

  Line() : SVGElement(LINE) {}
  Vector2D from;
  Vector2D to;
  void applyTransformation() override {
    doTransformation(from);
    doTransformation(to);
  }
  Line *copy() override {
    return new Line(*this);
  }
};

struct Polyline : SVGElement {

  Polyline() : SVGElement(POLYLINE) {}
  std::vector<Vector2D> points;
  void applyTransformation() override {
    for (auto &item : points) {
      doTransformation(item);
    }
  }
  Polyline *copy() override {
    return new Polyline(*this);
  }
};

struct Rect : SVGElement {

  Rect() : SVGElement(RECT) {}
  Vector2D position;
  Vector2D dimension;
  void applyTransformation() override {
    doTransformation(position);
    doTransformation(dimension);
  }
  Rect *copy() override {
    return new Rect(*this);
  }
};

struct Polygon : SVGElement {

  Polygon() : SVGElement(POLYGON) {}
  std::vector<Vector2D> points;
  void applyTransformation() override {
    for (auto &item : points) {
      doTransformation(item);
    }
  }
  Polygon *copy() override {
    return new Polygon(*this);
  }
};

struct Ellipse : SVGElement {

  Ellipse() : SVGElement(ELLIPSE) {}
  Vector2D center;
  Vector2D radius;
  void applyTransformation() override {
    doTransformation(center);
    doTransformation(radius);
  }
  Ellipse *copy() override {
    return new Ellipse(*this);
  }
};

struct Image : SVGElement {

  Image() : SVGElement(IMAGE) {}
  Vector2D position;
  Vector2D dimension;
  Texture tex;
  void applyTransformation() override {
    doTransformation(position);
    doTransformation(dimension);
  }
  Image *copy() override {
    return new Image(*this);
  }
};

struct SVG {

  ~SVG();
  float width, height;
  std::vector<SVGElement *> elements;
  SVG() : width(0), height(0) {}
  SVG(const SVG &svg) : width(svg.width), height(svg.height) {
    for (auto item : svg.elements) {
      elements.push_back(item->copy());
    }
  }
};

class SVGParser {
public:

  static int load(const char *filename, SVG *svg);
  static int save(const char *filename, const SVG *svg);

private:

  // parse a svg file
  static void parseSVG(XMLElement *xml, SVG *svg);

  // parse shared properties of svg elements
  static void parseElement(XMLElement *xml, SVGElement *element);

  // parse type specific properties
  static void parsePoint(XMLElement *xml, Point *point);
  static void parseLine(XMLElement *xml, Line *line);
  static void parsePolyline(XMLElement *xml, Polyline *polyline);
  static void parseRect(XMLElement *xml, Rect *rect);
  static void parsePolygon(XMLElement *xml, Polygon *polygon);
  static void parseEllipse(XMLElement *xml, Ellipse *ellipse);
  static void parseImage(XMLElement *xml, Image *image);
  static void parseGroup(XMLElement *xml, Group *group);

}; // class SVGParser

} // namespace CMU462

#endif // CMU462_SVG_H
