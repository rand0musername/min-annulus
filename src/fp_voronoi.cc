#include "fp_voronoi.h"
#include "voronoi_utils.h"

FarthestPointVoronoi::FarthestPointVoronoi(Model* model) {
    this->model = model;
    sites = model->GetPoints();
    dcel = model->GetFpVoronoiDcel();
}

std::future<void> FarthestPointVoronoi::ComputeDiagram() {
    return std::async(std::launch::async, &FarthestPointVoronoi::Incremental, this);
}

void FarthestPointVoronoi::ProcessAllCollinear() {
    int sz = sites.size();
    for (int i = 0; i < sz; i++) {
        sites[i].idx = i;
    }

    // Find extreme site
    geometry::Point min = sites[0];
    for (auto site : sites) {
        if (site.x < min.x || (site.x == min.x && site.y < min.y)) {
            min = site;
        }
    }

    // Find other extreme site
    geometry::Point max = sites[1];
    for (auto site : sites) {
        if (min.idx == site.idx) continue;
        if (geometry::Dist(site, min) > geometry::Dist(max, min)) {
            max = site;
        }
    }

    // This is needed for annulus finder
    hull.push_back(max);
    hull.push_back(min);
    model->SetHull(hull);

    inv.push_back(0);
    inv.push_back(1);

    // Take just these two
    {
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));

        dcel->faces.push_back(new Dcel::Face(0));  // min
        dcel->faces.push_back(new Dcel::Face(1));  // max
        open_face = new Dcel::Face(2);
        dcel->faces.push_back(open_face);  // outer
        geometry::Point mid = geometry::Midpoint(min, max);
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

        // Set origins
        upper_up->origin = v;
        lower_down->origin = v;

        // Set twins/prev/next
        v->incident_halfedge = upper_up;
        upper_up->twin = upper_down, upper_down->twin = upper_up;
        lower_up->twin = lower_down, lower_down->twin = lower_up;
        lower_up->next = upper_up, upper_up->prev = lower_up;
        upper_down->next = lower_down, lower_down->prev = upper_down;

        // Set incident faces
        upper_up->incident_face = dcel->faces[1];
        lower_up->incident_face = dcel->faces[1];
        upper_down->incident_face = dcel->faces[0];
        lower_down->incident_face = dcel->faces[0];

        // Add a line
        geometry::Line bis = geometry::Bisector(min, max);
        if (bis.vertical) {
            bis.dir = 'u';
            upper_up->line = bis;
            upper_down->line = bis;
            bis.dir = 'd';
            lower_up->line = bis;
            lower_down->line = bis;
        } else {
            char fst = (min.y < max.y) ? 'l' : 'r';
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

void FarthestPointVoronoi::ProcessRegular() {
    // Counterclockwise
    hull = geometry::GrahamScanConvexHull(sites);
    int hsz = hull.size();
    struct Node {
        int idx, idx_cw, idx_ccw;
        geometry::Point point;
    };

    // Init cw and ccw maps
    // Init indices
    for (int i = 0; i < hsz; i++) {
        hull[i].idx = i;
        ccw.push_back((i + 1) % hsz);
        cw.push_back((i + hsz - 1) % hsz);
    }

    model->SetHull(hull);
    // Shuffle the hull and fill inv map
    std::random_shuffle(hull.begin(), hull.end());
    inv.resize(hull.size());
    for (int i = 0; i < hsz; i++) {
        inv[hull[i].idx] = i;
    }

    for (int i = hsz - 1; i >= 4; i--) {
        int idx = hull[i].idx;
        // Kick node 'idx' and rewire around it
        ccw[cw[idx]] = ccw[idx];
        cw[ccw[idx]] = cw[idx];
    }

    // Add faces
    {
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));
        for (int i = 0; i < hsz; i++) {
            dcel->faces.push_back(new Dcel::Face(i));
        }
        open_face = new Dcel::Face(hsz);
        dcel->faces.push_back(open_face);  // outer face
    }

    // Compute initial solution and add points one-by-one
    ComputeInitialSolution(hull[0], hull[1], hull[2]);
    for (int i = 3; i < hsz; i++) {
        {
            std::lock_guard<std::mutex> lock(*(model->GetMutex()));
            AddPoint(hull, hull[i]);
        }
        Prune();  // Delete pruned vertices/half-edges
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
}

void FarthestPointVoronoi::Incremental() {
    // Incremental algorithm
    if (geometry::AllCollinear(sites)) {
        ProcessAllCollinear();
    } else {
        ProcessRegular();
    }

    // Add bounding box around
    Dcel::HalfEdge* open_edge;
    {
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));
        open_edge = voronoi_utils::AddBox(sites, open_face, dcel);

        // Fix outer/inner component pointers
        for (Dcel::HalfEdge* he : dcel->half_edges) {
            Dcel::Face* face = he->incident_face;
            if (face != open_face && face->outer_component == nullptr) {
                face->outer_component = open_edge;
                open_face->inner_components.push_back(he);
            }
        }
    }
    printf("Farthest-point Voronoi diagram found!\n");
}

void FarthestPointVoronoi::Prune() {
    {
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));

        // Prune edges
        std::vector<Dcel::HalfEdge*> new_edges;
        for (auto he : dcel->half_edges) {
            if (edges_pruned.find(he) == edges_pruned.end()) {
                new_edges.push_back(he);
            }
        }
        dcel->half_edges = new_edges;
        for (auto he : edges_pruned) delete he;
        edges_pruned.clear();

        // Prune vertices
        std::vector<Dcel::Vertex*> new_verts;
        for (auto v : dcel->vertices) {
            if (vertices_pruned.find(v) == vertices_pruned.end()) {
                new_verts.push_back(v);
            }
        }
        dcel->vertices = new_verts;
        for (auto v : vertices_pruned) delete v;
        vertices_pruned.clear();
    }
}

void FarthestPointVoronoi::AddPoint(const std::vector<geometry::Point>& hull, geometry::Point pt) {
    // Core logic: add a new point to the diagram

    Dcel::HalfEdge* curr = first_edge[ccw[pt.idx]];
    Dcel::HalfEdge *last_pt_fwd, *last_opt_fwd, *last_opt_bwd, *last_pt_bwd;
    last_pt_fwd = last_opt_bwd = last_opt_fwd = last_pt_bwd = nullptr;
    Dcel::Vertex* last_vertex = nullptr;
    geometry::Point opt;

    geometry::Point inter;
    while (true) {
        // Walk around faces until new face gets fully traced
        bool has_intersection;
        bool done = false;
        do {
            // Walk around one face until there is an intersection
            if (!edges_pruned.empty()) {
                if (curr->next == nullptr) {
                    done = true;
                    break;
                }
                curr = curr->next;
            }

            // Prune
            if (curr->origin != nullptr) {
                vertices_pruned.insert(curr->origin);
            }
            edges_pruned.insert(curr);
            edges_pruned.insert(curr->twin);
            opt = hull[inv[curr->incident_face->site]];  // also ccw[pt.idx]
            geometry::Line bis = geometry::Bisector(pt, opt);
            inter = geometry::LineIntersection(curr->line, bis);
            if (!curr->origin || !curr->twin->origin) {
                // Half infinite
                geometry::Point orig = (curr->origin) ? curr->origin->point : curr->twin->origin->point;
                has_intersection = geometry::CheckHalflineSide(inter, curr->line, orig);
            } else {
                // Regular
                has_intersection = geometry::CheckOrder(curr->origin->point, inter, curr->twin->origin->point);
            }
        } while (!has_intersection);
        if (done) {
            // This is the last iteration, finish the rewiring
            geometry::Point L;
            if (curr->origin != nullptr)
                L = curr->origin->point;
            else
                L = curr->twin->line.ForwardPoint(curr->twin->origin->point);
            geometry::Point R;
            if (curr->twin->origin != nullptr)
                R = curr->twin->origin->point;
            else
                R = curr->line.ForwardPoint(curr->origin->point);

            opt = hull[inv[cw[pt.idx]]];  // also ccw[pt.idx]

            // Add new edges
            Dcel::HalfEdge* pt_fwd = new Dcel::HalfEdge();
            first_edge[pt.idx] = pt_fwd;
            Dcel::HalfEdge* pt_bwd = new Dcel::HalfEdge();

            // Add pointers
            dcel->half_edges.push_back(pt_fwd);
            dcel->half_edges.push_back(pt_bwd);
            pt_bwd->origin = last_vertex;
            pt_fwd->twin = pt_bwd, pt_bwd->twin = pt_fwd;
            pt_fwd->incident_face = dcel->faces[pt.idx];
            pt_bwd->incident_face = dcel->faces[opt.idx];

            // Add a line
            geometry::Line bis = geometry::Bisector(pt, opt);
            if (bis.vertical) {
                bis.dir = 'd';
                geometry::Point nxt = bis.ForwardPoint(inter);
                if (geometry::Turn(L, R, nxt) == 1) {
                    bis.dir = 'u';
                }
            } else {
                bis.dir = 'l';
                geometry::Point nxt = bis.ForwardPoint(inter);
                if (geometry::Turn(L, R, nxt) == 1) {
                    bis.dir = 'r';
                }
            }
            pt_fwd->line = pt_bwd->line = bis;

            // Set next and prev
            pt_bwd->next = nullptr;
            pt_bwd->prev = last_opt_bwd;
            if (last_opt_bwd != nullptr) last_opt_bwd->next = pt_bwd;

            pt_fwd->next = last_pt_fwd;
            if (last_pt_fwd != nullptr) last_pt_fwd->prev = pt_fwd;
            pt_fwd->prev = nullptr;
            break;
        }

        // Add 1 vertex and 4 edges
        geometry::Point L;
        if (curr->origin != nullptr)
            L = curr->origin->point;
        else
            L = curr->twin->line.ForwardPoint(curr->twin->origin->point);
        geometry::Point R;
        if (curr->twin->origin != nullptr)
            R = curr->twin->origin->point;
        else
            R = curr->line.ForwardPoint(curr->origin->point);

        Dcel::Vertex* vertex = new Dcel::Vertex(inter);
        Dcel::HalfEdge* opt_fwd = new Dcel::HalfEdge();
        Dcel::HalfEdge* opt_bwd = new Dcel::HalfEdge();
        Dcel::HalfEdge* pt_fwd = new Dcel::HalfEdge();
        Dcel::HalfEdge* pt_bwd = new Dcel::HalfEdge();

        // Set pointers
        if (curr->twin->origin != nullptr) {
            curr->twin->origin->incident_halfedge = opt_bwd;
        }

        dcel->vertices.push_back(vertex);
        dcel->half_edges.push_back(pt_fwd);
        dcel->half_edges.push_back(pt_bwd);
        dcel->half_edges.push_back(opt_fwd);
        dcel->half_edges.push_back(opt_bwd);

        vertex->incident_halfedge = pt_fwd;
        pt_fwd->origin = opt_fwd->origin = vertex;
        opt_bwd->origin = curr->twin->origin;  // maybe nullptr
        pt_bwd->origin = last_vertex;

        opt_fwd->twin = opt_bwd, opt_bwd->twin = opt_fwd;
        pt_fwd->twin = pt_bwd, pt_bwd->twin = pt_fwd;

        pt_fwd->incident_face = dcel->faces[pt.idx];
        pt_bwd->incident_face = opt_fwd->incident_face = dcel->faces[opt.idx];
        opt_bwd->incident_face = dcel->faces[curr->twin->incident_face->site];

        // Add a line
        opt_bwd->line = opt_fwd->line = curr->line;

        // Bisector line
        geometry::Line bis = geometry::Bisector(pt, opt);
        if (bis.vertical) {
            bis.dir = 'd';
            geometry::Point nxt = bis.ForwardPoint(inter);
            if (geometry::Turn(L, R, nxt) == 1) {
                bis.dir = 'u';
            }
        } else {
            bis.dir = 'l';
            geometry::Point nxt = bis.ForwardPoint(inter);
            if (geometry::Turn(L, R, nxt) == 1) {
                bis.dir = 'r';
            }
        }
        pt_fwd->line = pt_bwd->line = bis;

        // Set next and prev
        pt_bwd->next = opt_fwd;
        opt_fwd->prev = pt_bwd;

        pt_bwd->prev = last_opt_bwd;
        if (last_opt_bwd != nullptr) last_opt_bwd->next = pt_bwd;

        opt_fwd->next = curr->next;
        if (curr->next != nullptr) curr->next->prev = opt_fwd;

        // prev taken care of
        pt_fwd->next = last_pt_fwd;
        if (last_pt_fwd != nullptr) last_pt_fwd->prev = pt_fwd;

        // pt_fwd->prev will be set later
        // opt_bwd->next will be set later
        opt_bwd->prev = curr->twin->prev;
        if (curr->twin->prev != nullptr) curr->twin->prev->next = opt_bwd;

        // first pt_bwd is definitely a first edge for ccw[pt]=opt
        if (last_pt_bwd == nullptr) first_edge[opt.idx] = pt_bwd;

        // every opt bwd is a first for someone if it has no origin
        if (opt_bwd->origin == nullptr) {
            int idx = opt_bwd->incident_face->site;
            first_edge[idx] = opt_bwd;
        }

        // Move on
        last_opt_fwd = opt_fwd;
        last_pt_fwd = pt_fwd;
        last_opt_bwd = opt_bwd;
        last_pt_bwd = pt_bwd;
        last_vertex = vertex;
        curr = curr->twin;
    }
}

void FarthestPointVoronoi::OrientBis(geometry::Line* bis, geometry::Point a, geometry::Point b) {
    if (bis->vertical) {
        if (a.x > b.x) {
            bis->dir = 'd';
        } else {
            bis->dir = 'u';
        }
    } else {
        if (a.y > b.y) {
            bis->dir = 'r';
        } else {
            bis->dir = 'l';
        }
    }
}

std::pair<Dcel::HalfEdge*, Dcel::HalfEdge*> FarthestPointVoronoi::AddHalfEdges(Dcel::Vertex* vertex,
                                                                               const geometry::Line& bis, int fst,
                                                                               int snd) {
    Dcel::HalfEdge* out = new Dcel::HalfEdge();
    Dcel::HalfEdge* in = new Dcel::HalfEdge();
    out->line = bis;
    in->line = bis;
    out->origin = vertex;
    out->twin = in;
    in->twin = out;

    out->incident_face = dcel->faces[fst];
    in->incident_face = dcel->faces[snd];

    dcel->half_edges.push_back(in);
    dcel->half_edges.push_back(out);
    return {in, out};
}

void FarthestPointVoronoi::ComputeInitialSolution(geometry::Point a, geometry::Point b, geometry::Point c) {
    if (geometry::Turn(a, b, c) == -1) {
        std::swap(b, c);
    }

    // Find circumcenter and take all three bisectors
    geometry::Point center = geometry::FindCircumcenter(a, b, c);
    geometry::Line ab_bis = geometry::Bisector(a, b);
    geometry::Line bc_bis = geometry::Bisector(b, c);
    geometry::Line ca_bis = geometry::Bisector(c, a);

    // orient bisectors properly
    OrientBis(&ab_bis, a, b);
    OrientBis(&bc_bis, b, c);
    OrientBis(&ca_bis, c, a);

    // Fill DCEL
    Dcel::Vertex* vertex = new Dcel::Vertex(center);
    {
        std::lock_guard<std::mutex> lock(*(model->GetMutex()));
        dcel->vertices.push_back(vertex);
        auto ab = AddHalfEdges(vertex, ab_bis, a.idx, b.idx);
        auto bc = AddHalfEdges(vertex, bc_bis, b.idx, c.idx);
        auto ca = AddHalfEdges(vertex, ca_bis, c.idx, a.idx);

        // ab/bc/ca = {in, out}
        first_edge[a.idx] = ca.first;
        first_edge[b.idx] = ab.first;
        first_edge[c.idx] = bc.first;
        vertex->incident_halfedge = ab.second;

        // Set next/prev pointers
        ab.first->next = bc.second;
        bc.second->prev = ab.first;

        bc.first->next = ca.second;
        ca.second->prev = bc.first;

        ca.first->next = ab.second;
        ab.second->prev = ca.first;
    }
}