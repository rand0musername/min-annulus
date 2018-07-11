#include "voronoi.h"
#include "voronoi_utils.h"

#include <cassert>

Voronoi::Voronoi(Model* model) : model(model) {
    sites = model->GetPoints();
    beach_line.SetSites(sites);
    dcel = model->GetVoronoiDcel();
    draw_beach_line = true;
}

std::future<void> Voronoi::ComputeDiagram() { return std::async(std::launch::async, &Voronoi::Fortunes, this); }

CircleEvent* Voronoi::DetectCircleEvent(LeafNode* a, LeafNode* b, LeafNode* c, double sw_y) {
    if (a->GetSite() == c->GetSite() || sites[b->GetSite()].y == sw_y) {
        return nullptr;
    }

    // Find parabola intersections and check the distance
    geometry::Point ab = geometry::FindParabolaIntersection(sites[a->GetSite()], sites[b->GetSite()], sw_y);
    geometry::Point bc = geometry::FindParabolaIntersection(sites[b->GetSite()], sites[c->GetSite()], sw_y);
    double dist = geometry::Dist(ab, bc);
    if (dist <= 1e-6) {
        return new CircleEvent(sw_y, ab, nullptr);  // Arc will be filled later
    }

    // Collinear, no circle event
    if (geometry::Turn(sites[a->GetSite()], sites[b->GetSite()], sites[c->GetSite()]) == 0) {
        return nullptr;
    }

    // There is potentially a circle event, find the circle
    geometry::Point center = geometry::FindCircumcenter(sites[a->GetSite()], sites[b->GetSite()], sites[c->GetSite()]);
    double radius = geometry::Dist(sites[a->GetSite()], center);
    double bottom_y = center.y - radius;

    // Check if the turns are right and the circle is not too high
    if (bottom_y >= sw_y) {
        return nullptr;
    }
    if (geometry::Turn(sites[a->GetSite()], sites[b->GetSite()], sites[c->GetSite()]) == 1) {
        return nullptr;
    }

    // We have a new circle event
    return new CircleEvent(bottom_y, center, nullptr);
}

void Voronoi::HandleInitialSiteEvent(const SiteEvent& event) {
    if (beach_line.GetRoot() == nullptr) {
        beach_line.SetRoot(new LeafNode(event.GetSite()));
        return;
    }
    LeafNode* first_leaf = beach_line.GetFirstLeaf();

    // Start tracing a new halfedge vertically to infinity
    geometry::Line line = geometry::Bisector(sites[event.GetSite()], sites[first_leaf->GetSite()]);
    line.dir = 'u';
    Dcel::HalfEdge* up = new Dcel::HalfEdge();
    Dcel::HalfEdge* down = new Dcel::HalfEdge();
    up->line = down->line = line;
    dcel->half_edges.push_back(up);
    dcel->half_edges.push_back(down);
    up->twin = down;
    down->twin = up;
    beach_line.InitialInsert(event.GetSite(), up);
}

void Voronoi::HandleSiteEvent(const SiteEvent& event) {
    LeafNode* arc_above = beach_line.FindArcAbove(sites[event.GetSite()].x, sites[event.GetSite()].y);

    if (arc_above->GetCircleEvent() != nullptr) {
        // False alarm
        arc_above->GetCircleEvent()->SetFalseAlarm(true);
    }

    // Start tracing a new half-edge
    Dcel::HalfEdge* upper = new Dcel::HalfEdge();
    Dcel::HalfEdge* lower = new Dcel::HalfEdge();
    geometry::Line line = geometry::Bisector(sites[event.GetSite()], sites[arc_above->GetSite()]);
    upper->line = lower->line = line;
    dcel->half_edges.push_back(upper);
    dcel->half_edges.push_back(lower);
    upper->twin = lower;
    lower->twin = upper;

    // Detect new circle events
    LeafNode* node = beach_line.Insert(arc_above, event.GetSite(), upper, lower);
    LeafNode* left = beach_line.FindPred(node);
    LeafNode* right = beach_line.FindSucc(node);
    LeafNode* far_left = beach_line.FindPred(left);
    LeafNode* far_right = beach_line.FindSucc(right);
    if (far_left != nullptr) {
        CircleEvent* circle_event = DetectCircleEvent(far_left, left, node, event.GetY());
        if (circle_event != nullptr) {
            circle_event->SetArc(left);
            left->SetCircleEvent(circle_event);
            event_queue.push(circle_event);
        }
    }
    if (far_right != nullptr) {
        CircleEvent* circle_event = DetectCircleEvent(node, right, far_right, event.GetY());
        if (circle_event != nullptr) {
            circle_event->SetArc(right);
            right->SetCircleEvent(circle_event);
            event_queue.push(circle_event);
        }
    }
}

void Voronoi::RefreshCircleEvent(LeafNode* arc, double sw_y) {
    // If there is a circle event mark it as a false alarm
    if (arc->GetCircleEvent() != nullptr) {
        arc->GetCircleEvent()->SetFalseAlarm(true);
        arc->GetCircleEvent()->SetArc(nullptr);
        arc->SetCircleEvent(nullptr);
    }

    // Find an new circle event
    LeafNode* pred = beach_line.FindPred(arc);
    LeafNode* succ = beach_line.FindSucc(arc);
    if (pred == nullptr || succ == nullptr) return;
    CircleEvent* circle_event = DetectCircleEvent(pred, arc, succ, sw_y);
    if (circle_event != nullptr) {
        circle_event->SetArc(arc);
        arc->SetCircleEvent(circle_event);
        event_queue.push(circle_event);
    }
}

void Voronoi::HandleCircleEvent(const CircleEvent& event) {
    // Find the affected site
    int site = event.GetArc()->GetSite();

    // We found a new DCEL vertex
    Dcel::Vertex* vertex = new Dcel::Vertex(event.GetCenter());
    dcel->vertices.push_back(vertex);

    // Find pred succ for circle event adding later
    LeafNode* pred = beach_line.FindPred(event.GetArc());
    LeafNode* succ = beach_line.FindSucc(event.GetArc());
    if (pred == nullptr || succ == nullptr) {
        assert(false);
    }

    // Start tracing a new edge down
    Dcel::HalfEdge* down = new Dcel::HalfEdge();
    Dcel::HalfEdge* up = new Dcel::HalfEdge();
    geometry::Line line = geometry::Bisector(sites[pred->GetSite()], sites[succ->GetSite()]);
    up->line = down->line = line;
    down->origin = vertex;
    down->incident_face = dcel->faces[pred->GetSite()];
    up->incident_face = dcel->faces[succ->GetSite()];
    vertex->incident_halfedge = down;
    down->twin = up;
    up->twin = down;
    dcel->half_edges.push_back(down);
    dcel->half_edges.push_back(up);

    // Delete from the beach line
    std::pair<Dcel::HalfEdge*, Dcel::HalfEdge*> edges = beach_line.Delete(event.GetArc(), up);

    // Refresh circle events where neighbourhood changed
    RefreshCircleEvent(pred, event.GetY());
    RefreshCircleEvent(succ, event.GetY());

    // Set origins and incident faces
    edges.first->origin = vertex;
    edges.first->incident_face = dcel->faces[site];
    edges.first->twin->incident_face = dcel->faces[pred->GetSite()];

    edges.second->origin = vertex;
    edges.second->incident_face = dcel->faces[succ->GetSite()];
    edges.second->twin->incident_face = dcel->faces[site];

    // Set next and prev pointrs
    edges.first->prev = edges.second->twin;
    edges.second->twin->next = edges.first;

    edges.second->prev = up;
    up->next = edges.second;

    down->prev = edges.first->twin;
    edges.first->twin->next = down;
}

void Voronoi::ProcessAllCollinear() {
    // Special case: all sites are collinear
    geometry::Point min = sites[0];
    for (auto site : sites) {
        if (site.x < min.x || (site.x == min.x && site.y < min.y)) {
            min = site;
        }
    }
    int sz = sites.size();
    for (int i = 0; i < sz; i++) {
        sites[i].idx = i;
    }

    // Sort by distance from the extreme line
    sort(sites.begin(), sites.end(), [min](const geometry::Point& a, const geometry::Point& b) {
        return geometry::Dist(min, a) < geometry::Dist(min, b);
    });
    {
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));
        for (int i = 0; i < sz - 1; i++) {
            // Add a new vertex and four new edges
            geometry::Point mid = geometry::Midpoint(sites[i], sites[i + 1]);
            Dcel::Vertex* v = new Dcel::Vertex(mid);
            dcel->vertices.push_back(v);
            Dcel::HalfEdge* upper_up = new Dcel::HalfEdge();
            Dcel::HalfEdge* upper_down = new Dcel::HalfEdge();
            Dcel::HalfEdge* lower_up = new Dcel::HalfEdge();
            Dcel::HalfEdge* lower_down = new Dcel::HalfEdge();
            dcel->half_edges.push_back(upper_up);
            dcel->half_edges.push_back(upper_down);
            dcel->half_edges.push_back(lower_up);
            dcel->half_edges.push_back(lower_down);

            // Set origin pointers
            upper_up->origin = v;
            lower_down->origin = v;

            // Set twin/next/prev pointers
            v->incident_halfedge = upper_up;
            upper_up->twin = upper_down, upper_down->twin = upper_up;
            lower_up->twin = lower_down, lower_down->twin = lower_up;
            lower_up->next = upper_up, upper_up->prev = lower_up;
            upper_down->next = lower_down, lower_down->prev = upper_down;

            // Set incident faces
            upper_up->incident_face = dcel->faces[sites[i + 1].idx];
            lower_up->incident_face = dcel->faces[sites[i + 1].idx];
            upper_down->incident_face = dcel->faces[sites[i].idx];
            lower_down->incident_face = dcel->faces[sites[i].idx];

            // Add the line
            geometry::Line bis = geometry::Bisector(sites[i], sites[i + 1]);
            if (bis.vertical) {
                bis.dir = 'u';
                upper_up->line = bis;
                upper_down->line = bis;
                bis.dir = 'd';
                lower_up->line = bis;
                lower_down->line = bis;
            } else {
                char fst = (sites[i].y < sites[i + 1].y) ? 'l' : 'r';
                char snd = (fst == 'l') ? 'r' : 'l';
                bis.dir = fst;
                upper_up->line = bis;
                upper_down->line = bis;
                bis.dir = snd;
                lower_up->line = bis;
                lower_down->line = bis;
            }
        }
    }
}

void Voronoi::ProcessEvents() {
    // Add site event to the event queue
    double max_y = sites[0].y;
    int sz = sites.size();
    for (int i = 0; i < sz; i++) {
        SiteEvent* event = new SiteEvent(sites[i].x, sites[i].y, i);
        event_queue.push(event);
        max_y = std::max(max_y, sites[i].y);
    }

    // Main event loop
    int events_done = 0;
    while (!event_queue.empty()) {
        Event* event = event_queue.top();
        model->SetSweepY(event->GetY());
        event_queue.pop();
        {
            std::lock_guard<std::mutex> lock(*(model->GetMutex()));
            if (model->GetSweepY() == max_y) {
                // This is a site event but the line never moved
                HandleInitialSiteEvent(*(static_cast<SiteEvent*>(event)));
            } else if (event->GetType() == 's') {
                // This is a site event
                HandleSiteEvent(*(static_cast<SiteEvent*>(event)));
            } else {
                // This is a circle event, we will discard it if it's a false alarm
                CircleEvent* circle_event = static_cast<CircleEvent*>(event);
                if (circle_event->GetFalseAlarm() == false) {
                    HandleCircleEvent(*circle_event);
                }
            }
        }
        delete event;
        events_done++;

        // Draw the beach line BST after each iteration
        beach_line.Draw("out/" + std::to_string(events_done));
    }

    {
        // Move the line a bit more and fix orientations
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));
        model->SetSweepY(model->GetSweepY() - 10);
        beach_line.SetOrientations(model->GetSweepY());
    }
}

void Voronoi::Fortunes() {
    // Add faces to DCEL
    int sz = sites.size();
    {
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));
        for (int i = 0; i < sz; i++) {
            dcel->faces.push_back(new Dcel::Face(i));
        }
        open_face = new Dcel::Face(sz);
        dcel->faces.push_back(open_face);  // outer face
    }

    // Process
    if (geometry::AllCollinear(sites)) {
        ProcessAllCollinear();
    } else {
        ProcessEvents();
    }

    {
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));

        // Add a bounding box around the diagram
        Dcel::HalfEdge* open_edge;
        open_edge = voronoi_utils::AddBox(sites, open_face, dcel);

        // Move the sweep line to its final position
        for (auto v : dcel->vertices) {
            model->SetSweepY(std::min(model->GetSweepY(), v->point.y - 10));
        }

        // Add inner/outer component pointers
        for (Dcel::HalfEdge* he : dcel->half_edges) {
            Dcel::Face* face = he->incident_face;
            if (face != open_face && face->outer_component == nullptr) {
                face->outer_component = open_edge;
                open_face->inner_components.push_back(he);
            }
        }
    }

    // TODO: If a Voronoi vertex is incident to four faces, merge two DCEL vertices with same coordinates
    printf("Voronoi diagram found!\n");
}