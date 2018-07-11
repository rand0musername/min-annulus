
#include "geometry.h"

#include <algorithm>
#include <cassert>
#include <cmath>

geometry::Line::Line() {}

geometry::Line::Line(double x) {
    this->vertical = true;
    this->x = x;
    this->k = this->n = 0;
    this->dir = 'r';
}

geometry::Line::Line(double k, double n) {
    this->vertical = false;
    this->k = k;
    this->n = n;
    this->x = 0;
    this->dir = 'r';
}

geometry::Point geometry::Line::ForwardPoint(Point start) {
    int offset = 100;
    if (vertical) {
        if (dir == 'u') {
            return {start.x, start.y + offset, 0};
        } else {
            return {start.x, start.y - offset, 0};
        }
    } else {
        double x = (dir == 'l') ? start.x - offset : start.x + offset;
        return {x, k * x + n, 0};
    }
}

geometry::Line geometry::Bisector(Point a, Point b) {
    // Special cases
    if (a.x == b.x) {
        return Line(0, (a.y + b.y) / 2);
    } else if (a.y == b.y) {
        return Line((a.x + b.x) / 2);
    }

    // Regular case, bisector equation
    Point mid({(a.x + b.x) / 2, (a.y + b.y) / 2, 0});
    double k = (a.y - b.y) / (a.x - b.x);
    k = (-1) / k;
    double n = mid.y - k * mid.x;
    return Line(k, n);
}

geometry::Point geometry::Midpoint(Point a, Point b) { return {(a.x + b.x) / 2, (a.y + b.y) / 2, 0}; }

bool geometry::ParallelLines(const Line& a, const Line& b) {
    if (a.vertical && b.vertical) return true;
    if (!a.vertical && !b.vertical && a.k == b.k) return true;
    return false;
}

geometry::Point geometry::LineIntersection(const Line& a, const Line& b) {
    assert(!ParallelLines(a, b));  // Should never happen

    double x, y;
    if (a.vertical) {
        x = a.x;
        y = b.k * x + b.n;
    } else if (b.vertical) {
        x = b.x;
        y = a.k * x + a.n;
    } else {
        x = (b.n - a.n) / (a.k - b.k);
        y = a.k * x + a.n;
    }
    return {x, y, 0};
}

double geometry::Dist(Point a, Point b) {
    double x_dist = (b.x - a.x) * (b.x - a.x);
    double y_dist = (b.y - a.y) * (b.y - a.y);
    return sqrt(x_dist + y_dist);
}

int geometry::Turn(Point a, Point b, Point c) {
    // 1=left, -1=right, 0=collinear
    double cross = (b.x - a.x) * (c.y - b.y) - (c.x - b.x) * (b.y - a.y);
    if (cross > 0) {
        return 1;
    } else if (cross < 0) {
        return -1;
    } else {
        return 0;
    }
}

int geometry::SameSide(Point a, Point b, Point c, Point d) {
    // 1=same, -1=diff, 0=a/b on cd
    return Turn(c, d, a) * Turn(c, d, b);
}

bool geometry::DoIntersect(Point a, Point b, Point c, Point d) {
    // Lines intersect with segments + the projections intersect
    return SameSide(a, b, c, d) <= 0 && SameSide(c, d, a, b) <= 0 && std::min(a.x, b.x) <= std::max(c.x, d.x) &&
           std::min(c.x, d.x) <= std::max(a.x, b.x) && std::min(a.y, b.y) <= std::max(c.y, d.y) &&
           std::min(c.y, d.y) <= std::max(a.y, b.y);
}

bool geometry::IsSelfIntersectingPoly(const std::vector<Point>& poly) {
    int n = poly.size();
    for (int i = 0; i < n; i++) {
        for (int j = i + 2; j < n; j++) {
            if (i == 0 && j == n - 1) continue;
            int i_nxt = (i + 1) % n;
            int j_nxt = (j + 1) % n;
            // Try to intersect two edges
            if (DoIntersect(poly[i], poly[i_nxt], poly[j], poly[j_nxt])) {
                return true;
            }
        }
    }
    return false;
}

double geometry::PolyArea(const std::vector<Point>& poly) {
    // Sum up the projections (shoelace formula)
    int n = poly.size();
    double area = 0;
    for (int i = 0; i < n; i++) {
        int nxt = (i + 1) % n;
        area += (poly[i].x + poly[nxt].x) * (poly[nxt].y - poly[i].y);
    }
    return fabs(area / 2);
}

geometry::Point geometry::FindCircumcenter(Point a, Point b, Point c) {
    Point center;

    double D = (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y);

    center.x = (((a.x - c.x) * (a.x + c.x) + (a.y - c.y) * (a.y + c.y)) / 2 * (b.y - c.y) -
                ((b.x - c.x) * (b.x + c.x) + (b.y - c.y) * (b.y + c.y)) / 2 * (a.y - c.y)) /
               D;

    center.y = (((b.x - c.x) * (b.x + c.x) + (b.y - c.y) * (b.y + c.y)) / 2 * (a.x - c.x) -
                ((a.x - c.x) * (a.x + c.x) + (a.y - c.y) * (a.y + c.y)) / 2 * (b.x - c.x)) /
               D;

    return center;
}

geometry::Point geometry::FindParabolaIntersection(Point L, Point R, double sw_y) {
    // Parabola vertices and focal lengths
    geometry::Point Lv({L.x, (sw_y + L.y) / 2, 0});
    geometry::Point Rv({R.x, (sw_y + R.y) / 2, 0});
    double Lf = fabs(L.y - Lv.y);
    double Rf = fabs(R.y - Rv.y);

    // If any f=0, it's a special case
    if (Lf == 0) {
        double x = Lv.x;
        double y = (x - Rv.x) * (x - Rv.x) / (4 * Rf) + Rv.y;
        return {x, y, 0};
    }
    if (Rf == 0) {
        double x = Rv.x;
        double y = (x - Lv.x) * (x - Lv.x) / (4 * Lf) + Lv.y;
        return {x, y, 0};
    }

    // Solve a quadratic equation
    double A = Rf - Lf;
    double B = 2 * (Lf * Rv.x - Rf * Lv.x);
    double C = Rf * Lv.x * Lv.x - Lf * Rv.x * Rv.x + 4 * Lf * Rf * (Lv.y - Rv.y);
    double x1, x2;

    // Rf == Lf
    if (A == 0) {
        if (L.x < R.x) {
            x1 = x2 = -C / B;
        } else {
            // This should never happen
            assert(false);
        }
    } else {
        // Two solutions
        double sqrtD = sqrt(B * B - 4 * A * C);
        x1 = (-B + sqrtD) / (2 * A);
        x2 = (-B - sqrtD) / (2 * A);
    }

    // Choose only one solution
    double x = (L.y > R.y) ? std::min(x1, x2) : std::max(x1, x2);
    double y = (x - Lv.x) * (x - Lv.x) / (4 * Lf) + Lv.y;
    return {x, y, 0};
}

geometry::Point geometry::RectHalfLineIntersection(Rect rect, Line line, Point origin) {
    // We need another point to use line intersection
    int offset = 10;
    Point other;
    if (line.vertical) {
        if (line.dir == 'u') {
            other = {line.x, rect.y2 + offset, 0};
        } else {
            other = {line.x, rect.y1 - offset, 0};
        }
    } else {
        if (line.dir == 'r') {
            other = {rect.x2 + offset, line.k * (rect.x2 + offset) + line.n, 0};
        } else {
            other = {rect.x1 - offset, line.k * (rect.x1 - offset) + line.n, 0};
        }
    }

    // Exactly one rect edge can be intersected since the origin is inside
    if (DoIntersect(origin, other, {rect.x1, rect.y2, 0}, {rect.x2, rect.y2, 0})) {
        return LineIntersection(line, {0, rect.y2});
    }
    if (DoIntersect(origin, other, {rect.x1, rect.y1, 0}, {rect.x2, rect.y1, 0})) {
        return LineIntersection(line, {0, rect.y1});
    }
    if (DoIntersect(origin, other, {rect.x1, rect.y1, 0}, {rect.x1, rect.y2, 0})) {
        return LineIntersection(line, {rect.x1});
    }
    if (DoIntersect(origin, other, {rect.x2, rect.y1, 0}, {rect.x2, rect.y2, 0})) {
        return LineIntersection(line, {rect.x2});
    }

    // This should never happen
    assert(false);
    return {0, 0, 0};
}

std::vector<geometry::Point> geometry::GrahamScanConvexHull(const std::vector<Point>& points) {
    // Helper struct
    struct HullPoint {
        Point p;
        double ang, dist;
        bool operator<(const HullPoint& b) const {
            if (ang == b.ang) return dist < b.dist;
            return ang < b.ang;
        }
    };

    // Find the starting point
    Point start = points[0];
    for (auto p : points) {
        if (p.y < start.y || (p.y == start.y && p.x < start.x)) {
            start = p;
        }
    }

    // Calculate angles and sort
    std::vector<HullPoint> all_pts;
    for (auto p : points) {
        double ang = atan2(p.y - start.y, p.x - start.x);
        double dist = Dist(p, start);
        all_pts.push_back({p, ang, dist});
    }
    sort(all_pts.begin(), all_pts.end());

    std::vector<Point> hull;
    hull.push_back(all_pts[0].p);
    hull.push_back(all_pts[1].p);

    int sz = points.size();
    for (int i = 2; i < sz; i++) {
        /*
            We observe the turn last two points make with the point we're trying to introduce
            While it's a right turn we discard the last point from the hull
            We keep discarding until we can safely add a[i]
            < - all points on the hull
            <= - the least number of points on the hull (no 3 collinear)
        */

        while (Turn(hull[hull.size() - 2], hull[hull.size() - 1], all_pts[i].p) <= 0) {
            hull.pop_back();
        }
        hull.push_back(all_pts[i].p);
    }
    return hull;
}

bool geometry::CheckHalflineSide(Point pt, Line l, Point orig) {
    switch (l.dir) {
        case 'u':
            return pt.y >= orig.y;
        case 'd':
            return pt.y <= orig.y;
        case 'l':
            return pt.x <= orig.x;
        case 'r':
            return pt.x >= orig.x;
    }

    // This should never happen
    assert(false);
    return false;
}

bool geometry::CheckOrder(Point a, Point b, Point c) {
    bool x = b.x >= std::min(a.x, c.x) && b.x <= std::max(a.x, c.x);
    bool y = b.y >= std::min(a.y, c.y) && b.y <= std::max(a.y, c.y);
    return x && y;
}

bool geometry::AllCollinear(const std::vector<Point>& points) {
    int sz = points.size();
    if (sz == 2) return true;  // Two points are always collinear
    for (int i = 2; i < sz; i++) {
        if (geometry::Turn(points[0], points[1], points[i]) != 0) return false;
    }
    return true;
}