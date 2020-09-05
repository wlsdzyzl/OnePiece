#ifndef GEOMETRY_2D_H
#define GEOMETRY_2D_H
#include "Geometry.h"
namespace fucking_cool 
{
namespace geometry 
{ 

    #define EPS (1e-5)
    
    struct LineSegment;
    struct Line;

    float Cross3(const Point2 &a, const Point2 &b, const Point2 &c);
    bool InSegBounding(const LineSegment &l, const Point2 &p);
    bool InSegBoundingX(const LineSegment &l, const Point2 &p);
    bool InSegBoundingY(const LineSegment &l, const Point2 &p);
    bool IsIntersecting(const LineSegment &l1, const LineSegment &l2);
    bool IsIntersecting(const Line & l1, const LineSegment &l2);
    Line LineFromSeg(const LineSegment &s);
    Point2 LineIntersect(const Line &a, const Line &b);
    Point2 SegIntersect(const LineSegment &s1, const LineSegment &s2);
    Point2 LineSegIntersect(const Line &a, const LineSegment & b);
    float Distance(const Point2 &a, const Point2 &b);
    Point2 ProjectionPointToLine(const Line &a, const Point2 &p);
    Point2 ProjectionPointToLineSegment(const LineSegment &a, const Point2 &p);
    int CheckPointInConvexPoly(const geometry::Point2List &points, const Point2 &p);
    int CheckPointToLine(const Line &line, const Point2 &point);
    int CheckPointToLine(const Point2 &a, const Point2 &b, const Point2 &z );
    int CheckPointInTriangle(const Point2 & a, const Point2 &b, const Point2 & c, const Point2 &p);
    float ComputeAreaTriangle(Point2 a, Point2 b, Point2 c);
    float ComputeAreaConvexPoly(const Point2List &points);
    struct LineSegment
    {
        Point2 p0, p1;
        LineSegment(){}
        LineSegment(Point2 _p0, Point2 _p1) : p0(_p0), p1(_p1) {}
        float Length()
        {
            return Distance(p0, p1);
        }
    };

    struct Line
    {
        geometry::Vector2 n;
        float d;
        Line() = default;
        Line(float a, float b, float c)
        {
            n(0) = a;
            n(1) = b;
            d = c;
        }
        Line(const geometry::Vector2 &_n, float _d)
        {
            n = _n;
            d = _d;
        }
        Line(const geometry::Point3 &l)
        {
            n = l.head<2>();
            d = l(2);
        }
    };

};
}
#endif
