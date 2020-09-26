#include "Geometry2d.h"
namespace fucking_cool
{
namespace geometry
{
    float Cross3(const Point2 &a, const Point2 &b, const Point2 &c)
    { 
        return (c(0) - b(0)) * (a(1) - b(1)) - (c(1) - b(1)) * (a(0) - b(0));
    }
    bool InSegBoundingX(const LineSegment &l, const Point2 &p)
    {
        if (std::min(l.p0(0), l.p1(0)) <= p(0) && p(0) <= std::max(l.p0(0), l.p1(0)))
            return 1;
        return 0;        
    }
    bool InSegBoundingY(const LineSegment &l, const Point2 &p)
    {
        if (std::min(l.p0(1), l.p1(1)) <= p(1) && p(1) <= std::max(l.p0(1), l.p1(1)))
            return 1;
        return 0;        
    }
    bool InSegBounding(const LineSegment &l, const Point2 &p)
    {
        if (std::min(l.p0(0), l.p1(0)) <= p(0) && p(0) <= std::max(l.p0(0), l.p1(0)) 
            && std::min(l.p0(1), l.p1(1)) <= p(1) && p(1) <= std::max(l.p0(1), l.p1(1)))
            return 1;
        return 0;
    }
    bool IsIntersecting(const Line & l1, const LineSegment &l2)
    {
        float min_x = std::min(l2.p0(0), l2.p1(0));
        float max_x = std::max(l2.p0(0), l2.p1(0));

        float y1 = (-l1.n(0) * min_x - l1.d) / l1.n(1);
        float y2 = (-l1.n(0) * max_x - l1.d) / l1.n(1);
        
        LineSegment l(Point2(min_x,y1), Point2(max_x, y2));

        return IsIntersecting(l,l2);
    }
    bool IsIntersecting(const LineSegment &l1, const LineSegment &l2)
    {
        float d1 = Cross3(l1.p0, l1.p1, l2.p0);
        float d2 = Cross3(l1.p0, l1.p1, l2.p1);
        float d3 = Cross3(l2.p0, l2.p1, l1.p0);
        float d4 = Cross3(l2.p0, l2.p1, l1.p1);
        
        if (((d1 < -EPS && d2 > EPS) || (d1 > EPS && d2 < -EPS)) && ((d3 < -EPS && d4 > EPS) || (d3 > EPS && d4 < -EPS)))
            return 1;
        if ((fabs(d1) <= EPS && InSegBounding(l1, l2.p0)) || (fabs(d2) <= EPS && InSegBounding(l1, l2.p1)) 
            || (fabs(d3) <= EPS && InSegBounding(l2, l1.p0)) || (fabs(d4) <= EPS && InSegBounding(l2, l1.p1)))
            return 1;
        return 0;
    }

    Line LineFromSeg(const LineSegment &s)
    {
        Line line;
        line.n = geometry::Vector2(s.p0(1) - s.p1(1), s.p1(0) - s.p0(0));
        line.n.normalize();
        line.d = - line.n.transpose() * s.p0;
        return line;
    }
    Point2 LineIntersect(const Line &a, const Line &b)
    {
        float x = a.n(1) * b.d - b.n(1) * a.d;
        float y = b.n(0) * a.d - a.n(0) * b.d;
        return Point2(x,y) / (a.n(0) * b.n(1) - b.n(0) * a.n(1));
    }
    Point2 SegIntersect(const LineSegment &s1, const LineSegment &s2)
    {
        Line l1 = LineFromSeg(s1);
        Line l2 = LineFromSeg(s2);
        return LineIntersect(l1, l2);
    }
    Point2 LineSegIntersect(const Line &a, const LineSegment & b)
    {
        Line l = LineFromSeg(b);
        return LineIntersect(a,l);
    }
    float Distance(const Point2 &a, const Point2 &b)
    {
        return (a - b).norm();
    }
    Point2 ProjectionPointToLine(const Line &a, const Point2 &p)
    {
        float tmp = a.n.transpose() * a.n;
        return  Point2( (a.n(1) * a.n(1) * p(0) - a.n(0) * a.n(1) * p(1) - a.n(0)* a.d)/ tmp,
         (a.n(0) * a.n(0) * p(1) -a.n(0) * a.n(1) * p(0) - a.n(1) * a.d) / tmp ) ;
    }
    Point2 ProjectionPointToLineSegment(const LineSegment &a, const Point2 &p)
    {
        return ProjectionPointToLine(LineFromSeg(a), p);
    }

    int CheckPointToLine(const Line &line, const Point2 &point)
    {
        float distance = line.n.transpose() * point + line.d;
        if(std::fabs(distance) < EPS) return 0;
        if(distance > 0) return 1;
        return -1;
        //if(distance == 0) return 0;
        //return distance; 
    }
    int CheckPointToLine(const Point2 &a, const Point2 &b, const Point2 &z )
    {
        // equal to CheckPointToLine(LineFromSeg(LineSegment(a,b)), z);
        float d =  a(0)* b(1) + a(1) * z(0) + b(0) * z(1) - z(0)* b(1) - a(1)*b(0) - a(0) * z(1);
        //std::cout<<d<<std::endl;
        if(std::fabs(d) < EPS) return 0;
        if(d > 0) return 1;
        return -1;
    }
    int CheckPointInTriangle(const Point2 & a, const Point2 &b, const Point2 & c, const Point2 &p)
    {
        // Check if a point is in the triangle. 
        // And we consider "the point is on the triangle" as "the point is in the triangle"
        int a_r = CheckPointToLine(a, b, p);
        int b_r = CheckPointToLine(b, c, p);
        int c_r = CheckPointToLine(c, a, p);
        //std::cout<<a_r<<" "<<b_r<<" "<<c_r<<std::endl;
        if(a_r == b_r && b_r == c_r)
        return 1;
        if((a_r == b_r && c_r == 0) || (a_r == c_r && b_r == 0) || (c_r == b_r && a_r == 0))
        return 1;
        if((a_r == b_r && a_r == 0) || (a_r == c_r && a_r == 0) || (c_r == b_r && c_r == 0))
        return 1;

        return 0;
    }
    float ComputeAreaTriangle(Point2 a, Point2 b, Point2 c)
    {
        return std::fabs(0.5*(a(0) * b(1) + b(0) * c(1) + c(0) * a(1) - 
            a(0) * c(1) - b(0) * a(1) - c(0) * b(1) ));
    }
    float ComputeAreaConvexPoly(const Point2List &points)
    {
        int seed = 0; 
        int start = 1;
        int end = points.size();
        float area = 0;
        if(points.size() < 3) return 0;

        for(int i = start; i < end-1; ++i)
        {
            area += ComputeAreaTriangle(points[seed], points[i], points[i + 1]);
        }
        return area;
    }
    int CheckPointInConvexPoly(const geometry::Point2List &points, const Point2 &p )
    {
        // Same as the in-triangle-test
        int seed = 0;
        int start = 1;
        int end = points.size(); 
        int check = (start + end )/2;        
        if(end < 3) 
        {
            std::cout<<YELLOW<<"[CheckPointInConvexPoly]::[Warning]::Not a convex polygon."<<RESET<<std::endl;
            return -1;
        }

        while(true)
        {
            if(end - start == 2)
            {
                return CheckPointInTriangle(points[seed], points[start], points[end-1], p);
            }
            if(end <= start)
            {
                std::cout<<RED<<"[CheckPointInConvexPoly]::[ERROR]::Something wrong."<<RESET<<std::endl;
                //std::cout<<end<<" "<<start<<std::endl;
                return 0;                
            }

            int d = CheckPointToLine(points[seed], points[check], p);
            //CheckPointToLine(points[check],points[seed],p);
            if(d == 1)
            {
                start = check;
            }
            else
            {
                end = check + 1;
            }
            check = (start + end )/2;
        }
        return 0;
    }
}
}
