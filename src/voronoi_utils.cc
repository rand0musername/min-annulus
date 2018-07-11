#include "voronoi_utils.h"

#include <algorithm>
#include <cmath>

Dcel::HalfEdge* voronoi_utils::AddBox(const std::vector<geometry::Point>& sites, Dcel::Face* open_face, Dcel* dcel) {
    // Calculate box vertices
    geometry::Rect box({sites[0].x, sites[0].x, sites[0].y, sites[0].y});
    for (auto site : sites) {
        box.x1 = std::min(box.x1, site.x);
        box.y1 = std::min(box.y1, site.y);
        box.x2 = std::max(box.x2, site.x);
        box.y2 = std::max(box.y2, site.y);
    }
    for (auto vertex : dcel->vertices) {
        const geometry::Point& site = vertex->point;
        box.x1 = std::min(box.x1, site.x);
        box.y1 = std::min(box.y1, site.y);
        box.x2 = std::max(box.x2, site.x);
        box.y2 = std::max(box.y2, site.y);
    }

    // Make it a bit wider
    box.x1 -= 50, box.y1 -= 50;
    box.x2 += 50, box.y2 += 50;
    double size = std::max(box.y2 - box.y1, box.x2 - box.x1);
    double x_diff = size - (box.x2 - box.x1);
    box.x1 -= x_diff / 2, box.x2 += x_diff / 2;
    double y_diff = size - (box.y2 - box.y1);
    box.y1 -= y_diff / 2, box.y2 += y_diff / 2;

    // Create new vertices
    std::vector<Dcel::Vertex*> box_verts;
    box_verts.push_back(new Dcel::Vertex({box.x1, box.y1, 0}, true));
    box_verts.push_back(new Dcel::Vertex({box.x2, box.y2, 0}, true));
    box_verts.push_back(new Dcel::Vertex({box.x1, box.y2, 0}, true));
    box_verts.push_back(new Dcel::Vertex({box.x2, box.y1, 0}, true));

    // Find all edges with no origin and intersect them with the box
    for (auto edge : dcel->half_edges) {
        if ((edge->twin->origin) == nullptr) {
            geometry::Point inter = geometry::RectHalfLineIntersection(box, edge->line, edge->origin->point);

            // Intersection gives a new vertex
            Dcel::Vertex* vert = new Dcel::Vertex(inter, true);
            bool new_vert = true;
            for (auto vert2 : box_verts) {
                if (vert2->point.x == vert->point.x && vert2->point.y == vert->point.y) {
                    vert = vert2;
                    new_vert = false;
                    break;
                }
            }
            edge->twin->origin = vert;
            vert->incident_halfedge = edge->twin;
            if (new_vert) box_verts.push_back(vert);
        }
    }

    // Circular sort of box vertices
    geometry::Point mid = {(box.x1 + box.x2) / 2, (box.y1 + box.y2) / 2, 0};
    std::sort(box_verts.begin(), box_verts.end(), [mid](const Dcel::Vertex* a, const Dcel::Vertex* b) {
        double ang_a = atan2(mid.y - a->point.y, mid.x - a->point.x);
        double ang_b = atan2(mid.y - b->point.y, mid.x - b->point.x);
        return ang_a > ang_b;
    });

    // Do one full circle and create edges
    int sz = box_verts.size();
    std::vector<Dcel::HalfEdge*> fwds;
    std::vector<Dcel::HalfEdge*> bwds;
    for (int i = 0; i < sz; i++) {
        fwds.push_back(new Dcel::HalfEdge());
        bwds.push_back(new Dcel::HalfEdge());
    }

    // Connect box edges
    for (int i = 0; i < sz; i++) {
        // Add vertex to DCEL
        dcel->vertices.push_back(box_verts[i]);
        int i_nxt = (i + 1) % sz;
        int i_prev = (i + sz - 1) % sz;

        // Add edges to DCEL
        Dcel::HalfEdge* fwd = fwds[i];
        Dcel::HalfEdge* bwd = bwds[i];
        dcel->half_edges.push_back(fwd);
        dcel->half_edges.push_back(bwd);

        // Set twin and origin pointers
        fwd->twin = bwd, bwd->twin = fwd;
        fwd->origin = box_verts[i];
        bwd->origin = box_verts[i_nxt];

        // Set incident_face pointers
        bwd->incident_face = open_face;
        int idx = i;
        while (box_verts[idx]->incident_halfedge == nullptr) {
            idx = (idx + sz - 1) % sz;
        }
        fwd->incident_face = box_verts[idx]->incident_halfedge->twin->incident_face;

        // Set next and prev pointers
        bwds[i_prev]->prev = bwds[i];
        bwds[i]->next = bwds[i_prev];
        if (box_verts[i]->incident_halfedge == nullptr) {
            // Corner, special case
            fwds[i]->prev = fwds[i_prev];
            fwds[i_prev]->next = fwds[i];
            box_verts[i]->incident_halfedge = fwds[i];
        } else {
            // Not a corner
            fwds[i]->prev = box_verts[i]->incident_halfedge->twin;
            box_verts[i]->incident_halfedge->twin->next = fwds[i];

            fwds[i_prev]->next = box_verts[i]->incident_halfedge;
            box_verts[i]->incident_halfedge->prev = fwds[i_prev];
        }
    }
    // A half-edge incident to open_face
    return bwds[0];
}