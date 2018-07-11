#pragma once
#include <vector>

namespace geometry {
struct Point {
    double x, y;
    int idx;
};

struct Line {
    Line();
    Line(double x);            // vertical
    Line(double k, double n);  // non-vertical

    bool vertical;
    double x;     // vertical
    double k, n;  // non-vertical
    char dir;     // 'l' 'r' 'u' 'd'

    // TODO: make new classes for vert/non-vert

    // Returns a point somewhere along direction 'dir'
    Point ForwardPoint(Point start);
};

struct Rect {
    double x1, x2, y1, y2;
};

struct Annulus {
    Point center;
    double r_inner;
    double r_outer;
    Annulus() { r_inner = r_outer = -1; }
};

// Perpendicular bisector for two points
Line Bisector(Point a, Point b);

// A middle point of the connecting line segment
Point Midpoint(Point a, Point b);

// Checks if two lines are parallel
bool ParallelLines(const Line& a, const Line& b);

// Intersection of two undirected lines
Point LineIntersection(const Line& a, const Line& b);

// Calculates Euclidean distance
double Dist(Point a, Point b);

// Returns the turn given by a->b->c using a cross product
// 1=left, -1=right, 0=collinear
int Turn(Point a, Point b, Point c);

// Checks if a and b are on the same side of (c, d)
// 1=same, -1=diff, 0=a/b on cd
int SameSide(Point a, Point b, Point c, Point d);

// Checks if line segments ab and cd intersect
bool DoIntersect(Point a, Point b, Point c, Point d);

// Checks if a polygon is self intersecting
bool IsSelfIntersectingPoly(const std::vector<Point>& poly);

// Calculates the area of a polygon by summing intersections
double PolyArea(const std::vector<Point>& poly);

// Finds circumcircle of the triangle
Point FindCircumcenter(Point a, Point b, Point c);

// Intersects two parabolas for a given sweep line position
Point FindParabolaIntersection(Point a, Point b, double line_y);

// Intersects a directed half-line with a rectangle
Point RectHalfLineIntersection(Rect rect, Line line, Point origin);

// Finds a convex hull for a set of points
std::vector<Point> GrahamScanConvexHull(const std::vector<Point>& points);

// Checks if a point on the line is in the right direction
bool CheckHalflineSide(Point pt, Line l, Point orig);

// Check the order of collinear points
bool CheckOrder(Point a, Point b, Point c);

// Checks if all points in the set are collinear
bool AllCollinear(const std::vector<Point>& points);
}  // namespace geometry