#include "dcel.h"
#include <cstdio>

Dcel::~Dcel() {
    for (auto v : vertices) delete v;
    for (auto he : half_edges) delete he;
    for (auto f : faces) delete f;
}

void Dcel::Print() {
    printf("=== Printing DCEL ===\n");
    printf("Vertices:\n");
    for (Vertex* v : vertices) {
        printf("point=(%.2f %.2f) incident_halfedge=\n", v->point.x, v->point.y);
        if (v->incident_halfedge == nullptr)
            printf("NONE\n");
        else
            v->incident_halfedge->Print();
    }
    printf("Faces:\n");
    for (Face* f : faces) {
        printf("site=%d ", f->site);
        if (f->outer_component != nullptr) {
            printf("outer component: ((%.2f %.2f)->(%.2f %.2f)) ", f->outer_component->origin->point.x,
                   f->outer_component->origin->point.y, f->outer_component->twin->origin->point.x,
                   f->outer_component->twin->origin->point.y);
        }
        if (f->inner_components.size() > 0) {
            printf("inner components: ");
            for (HalfEdge* he : f->inner_components) {
                he->Print();
                printf("\nic: ((%.2f %.2f)->(%.2f %.2f)) ", he->origin->point.x, he->origin->point.y,
                       he->twin->origin->point.x, he->twin->origin->point.y);
            }
        }
        printf("\n");
    }
    printf("HalfEdges:\n");
    for (HalfEdge* he : half_edges) {
        he->Print();
        printf("Line: vertical = %d | (k, n, x) = (%.2f %.2f %.2f) | dir = %c\n", he->line.vertical, he->line.k,
               he->line.n, he->line.x, he->line.dir);
        printf("Neighbours!\n");
        if (he->next != nullptr) {
            printf("Next:\n");
            he->next->Print();
        } else {
            printf("Next: NO\n");
        }
        if (he->prev != nullptr) {
            printf("Prev:\n");
            he->prev->Print();
        } else {
            printf("Prev: NO\n");
        }
        printf("\n");
    }
    printf("=== Done printing DCEL ===\n");
}

void Dcel::HalfEdge::Print() {
    // Origins
    if (origin != nullptr)
        printf("((%.2f %.2f)->", origin->point.x, origin->point.y);
    else
        printf("((NO_ORIGIN)->");

    if (twin != nullptr && twin->origin != nullptr)
        printf("(%.2f %.2f))\n", twin->origin->point.x, twin->origin->point.y);
    else
        printf("((NO_TWIN_ORIGIN))\n");

    if (incident_face != nullptr)
        printf("incident_face=%d\n", incident_face->site);
    else
        printf("incident_face=NONE\n");
}