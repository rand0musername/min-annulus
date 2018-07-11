#include "annulus_finder.h"

#include "dcel.h"
#include "geometry.h"

AnnulusFinder::AnnulusFinder(std::future<void>* fut1, std::future<void>* fut2, Model* model) {
    this->fut1 = fut1;
    this->fut2 = fut2;
    this->model = model;
}

std::future<void> AnnulusFinder::FindAnnulus() {
    // Launch a new thread
    return std::async(std::launch::async, &AnnulusFinder::MergeAndFind, this);
}

void AnnulusFinder::MergeAndFind() {
    // Wait for threads to finish
    fut1->get();
    fut2->get();

    // Initialize locators
    voronoi_pl.LoadDcel(model->GetVoronoiDcel());
    fp_voronoi_pl.LoadDcel(model->GetFpVoronoiDcel());

    // Find the best candidate
    GenerateCandidates();
    model->FindBestAnnulus();
    printf("Annulus Finder done!\n");
    double roundness = model->GetAnnulus()->r_outer - model->GetAnnulus()->r_inner;
    printf("Roundness = %.2f\n", roundness < 1e-6 ? 0 : roundness);
}

void AnnulusFinder::GenerateCandidates() {
    // Candidate type 1: Voronoi vertices
    for (Dcel::Vertex* vert : model->GetVoronoiDcel()->vertices) {
        // Ignore box vertices
        if (vert->box) continue;

        // We need any half-edge since the distances are the same
        Dcel::HalfEdge* he = vert->incident_halfedge;
        int idx = he->incident_face->site;

        // Build the annulus
        geometry::Annulus ann;
        ann.center = vert->point;
        ann.r_inner = geometry::Dist(ann.center, model->GetPoint(idx));
        geometry::Point farthest = model->GetHullPoint(fp_voronoi_pl.Locate(ann.center));
        ann.r_outer = geometry::Dist(ann.center, farthest);
        {
            std::lock_guard<std::mutex> lock(*(model->GetMutex()));
            model->AddAnnCandidate(ann);
        }
    }

    // Candidate type 2: farthest-point Voronoi vertices
    for (Dcel::Vertex* vert : model->GetFpVoronoiDcel()->vertices) {
        // Ignore box vertices
        if (vert->box) continue;

        // We need any half-edge since the distances are the same
        Dcel::HalfEdge* he = vert->incident_halfedge;
        int idx = he->incident_face->site;

        // Build the annulus
        geometry::Annulus ann;
        ann.center = vert->point;
        ann.r_outer = geometry::Dist(ann.center, model->GetHullPoint(idx));
        geometry::Point closest = model->GetPoint(voronoi_pl.Locate(ann.center));
        ann.r_inner = geometry::Dist(ann.center, closest);
        {
            std::lock_guard<std::mutex> lock(*(model->GetMutex()));
            model->AddAnnCandidate(ann);
        }
    }

    // Candidate type 3: Edge intersections
    for (Dcel::HalfEdge* he1 : model->GetVoronoiDcel()->half_edges) {
        // Ignore box edges and duplicates
        if (he1->incident_face < he1->twin->incident_face) continue;
        if (he1->origin->box && he1->twin->origin->box) continue;

        for (Dcel::HalfEdge* he2 : model->GetFpVoronoiDcel()->half_edges) {
            // Ignore box edges and duplicates
            if (he2->incident_face < he2->twin->incident_face) continue;
            if (he2->origin->box && he2->twin->origin->box) continue;

            // Orient halflines properly
            if (he1->origin->box) he1 = he1->twin;
            if (he2->origin->box) he2 = he2->twin;

            // Process 4 cases (halfline/segment X halfline/segment)
            if (geometry::ParallelLines(he1->line, he2->line)) continue;
            geometry::Point inter = geometry::LineIntersection(he1->line, he2->line);
            bool has_intersection = true;
            if (he1->twin->origin->box) {
                // Halfline
                has_intersection &= geometry::CheckHalflineSide(inter, he1->line, he1->origin->point);
            } else {
                // Segment
                has_intersection &= geometry::CheckOrder(he1->origin->point, inter, he1->twin->origin->point);
            }
            if (he2->twin->origin->box) {
                // Halfline
                has_intersection &= geometry::CheckHalflineSide(inter, he2->line, he2->origin->point);
            } else {
                // Segment
                has_intersection &= geometry::CheckOrder(he2->origin->point, inter, he2->twin->origin->point);
            }
            if (!has_intersection) continue;

            // Build the annulus
            geometry::Annulus ann;
            ann.center = inter;
            ann.r_inner = geometry::Dist(ann.center, model->GetPoint(he1->incident_face->site));
            ann.r_outer = geometry::Dist(ann.center, model->GetHullPoint(he2->incident_face->site));
            {
                std::lock_guard<std::mutex> lock(*(model->GetMutex()));
                model->AddAnnCandidate(ann);
            }
        }
    }
}
