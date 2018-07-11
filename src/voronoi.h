#pragma once
#include <queue>
#include <vector>
#include "beach_line.h"
#include "dcel.h"
#include "event.h"
#include "geometry.h"
#include "model.h"

class Voronoi {
   public:
    Voronoi(Model* model);

    // Compute the diagram in a new thread
    std::future<void> ComputeDiagram();

   private:
    // Find a circle event defined by arcs (a, b, c) for a fixed sweep line position
    CircleEvent* DetectCircleEvent(LeafNode* a, LeafNode* b, LeafNode* c, double sw_y);

    // Handle site events
    void HandleInitialSiteEvent(const SiteEvent& event);
    void HandleSiteEvent(const SiteEvent& event);

    // Delete the circle event anchored at an arc if it's stale and create a new one
    void RefreshCircleEvent(LeafNode* arc, double sw_y);

    // Handle circle events
    void HandleCircleEvent(const CircleEvent& event);

    // Special case: all sites are collinear
    void ProcessAllCollinear();

    // Main event loop
    void ProcessEvents();

    // Fortune's algorithm for Voronoi diagram computation
    void Fortunes();

    // Model
    Model* model;

    struct Cmp {
        bool operator()(Event* arg1, Event* arg2) {
            return *arg1 < *arg2;  // Decreasing y
        }
    };
    std::priority_queue<Event*, std::vector<Event*>, Cmp> event_queue;
    BeachLine beach_line;
    Dcel::Face* open_face;  // Unbounded face
    std::vector<geometry::Point> sites;
    Dcel* dcel;

    // If set, will save a graphviz graph representation of the
    // beach line BST after each Fortune's step
    double draw_beach_line;
};