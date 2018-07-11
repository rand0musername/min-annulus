#include "point_locator.h"

#include <algorithm>

void PointLocator::LoadDcel(Dcel* dcel) {
    // Init all possible slabs and add one extra slab at the end
    double max_x = dcel->vertices[0]->point.x;
    for (Dcel::Vertex* v : dcel->vertices) {
        if (v->box) continue;
        slabs[v->point.x] = {};
        max_x = std::max(max_x, v->point.x);
    }
    int offset = 100;
    double last_slab_x = max_x + offset;
    slabs[last_slab_x] = {};

    // Binary search for each edge
    verticals = true;
    for (Dcel::HalfEdge* he : dcel->half_edges) {
        // Avoid doubles and box edges
        if (he->incident_face < he->twin->incident_face) continue;
        if (he->origin->box && he->twin->origin->box) continue;

        if (!he->origin->box && !he->twin->origin->box) {
            // One point is on the box: half-infinite edge
            if (he->origin->point.x > he->twin->origin->point.x) he = he->twin;

            // Find first y bigger or equal = first slab I intersect
            auto it = slabs.lower_bound(he->origin->point.x);
            ++it;
            double last = it->first;
            double R = he->twin->origin->point.x;

            // Add to all slabs
            while (it != slabs.end() && (last < R || std::fabs(last - R) < 1e-6)) {
                it->second.push_back({he->line, he->incident_face->site, he->twin->incident_face->site});
                verticals = false;
                ++it;
                last = it->first;
            }
        } else {
            // A regular edge inside the box
            if (he->origin->box) he = he->twin;
            if (he->line.vertical) {
                // A single vertical line
                // If all are vertical, there are only two farthest-point faces 
                vert_L = (he->line.dir == 'd') ? he->incident_face->site : he->twin->incident_face->site;
                vert_R = (he->line.dir == 'u') ? he->incident_face->site : he->twin->incident_face->site;
                vert_thresh = he->line.x;
                continue;
            }

            // Find first y bigger or equal = first slab I intersect
            auto it = slabs.lower_bound(he->origin->point.x);
            if (he->line.dir == 'r') ++it;
            while (it != slabs.end()) {
                // Find surrounding sites
                int site_below = (he->line.dir == 'r') ? he->incident_face->site : he->twin->incident_face->site;
                int site_above = (he->line.dir == 'l') ? he->incident_face->site : he->twin->incident_face->site;
                it->second.push_back({he->line, site_below, site_above});
                verticals = false;

                // Move in a right direction
                if (he->line.dir == 'r') {
                    ++it;
                } else {
                    if (it == slabs.begin()) break;
                    --it;
                }
            }
        }
    }

    if (verticals) {
        // All done if they were vertical, will be handed separately
        return;
    }

    // Sort lines in every individual slab by y
    double last_x;
    bool fst = true;
    for (std::pair<const double, std::vector<Info>>& kv : slabs) {
        double x = kv.first;
        if (fst) last_x = x - 10;
        fst = false;
        std::sort(kv.second.begin(), kv.second.end(), [last_x, x](const Info& a, const Info& b) {
            double ay = a.line.k * x + a.line.n;
            double by = b.line.k * x + b.line.n;
            if (fabs(ay - by) < 1e-6) {
                double ay2 = a.line.k * last_x + a.line.n;
                double by2 = b.line.k * last_x + b.line.n;
                return ay2 < by2;
            } else {
                return ay < by;
            }
        });
        last_x = x;
    }
}

int PointLocator::Locate(geometry::Point pt) {
    // Locate a given point
    if (verticals) {
        // If all are vertical, there are only two farthest-point faces 
        return (pt.x <= vert_thresh) ? vert_L : vert_R;
    }

    // Binary search the right slab
    auto it = slabs.lower_bound(pt.x);
    if (it == slabs.end()) --it;
    auto vec = it->second;
    int sz = vec.size();

    // Binary search in a slab, look for the last line that's above
    int lo = 0, hi = sz - 1, pivot;
    while (lo < hi) {
        pivot = (lo + hi) / 2;
        auto line = vec[pivot].line;
        double y = line.k * pt.x + line.n;
        if (y >= pt.y)
            hi = pivot;
        else
            lo = pivot + 1;
    }

    // Return the site tied to the face
    auto line = vec[lo].line;
    double y = line.k * pt.x + line.n;
    if (y > pt.y)
        return vec[lo].site_below;
    else
        return vec.back().site_above;
}
