#pragma once
#include <map>
#include <set>
#include <vector>
#include "dcel.h"
#include "geometry.h"
#include "model.h"

class FarthestPointVoronoi {
   public:
    FarthestPointVoronoi(Model* model);

    // Find the farthest-point Voronoi diagram in a new thread
    std::future<void> ComputeDiagram();

   private:
    // Special case: all sites collinear
    void ProcessAllCollinear();

    // Regular case
    void ProcessRegular();

    // Incremental algorithm for farthest-point Voronoi diagram construction
    void Incremental();

    // Prune deleted vertices/edges
    void Prune();

    // Add a new point
    void AddPoint(const std::vector<geometry::Point>& hull, geometry::Point pt);

    // Orient the bisector
    void OrientBis(geometry::Line* bis, geometry::Point a, geometry::Point b);

    // A helper function to help deal with half-edges
    std::pair<Dcel::HalfEdge*, Dcel::HalfEdge*> AddHalfEdges(Dcel::Vertex* vertex, const geometry::Line& bis, int fst,
                                                             int snd);

    // Finds the initial solution for three points
    void ComputeInitialSolution(geometry::Point a, geometry::Point b, geometry::Point c);

    Dcel::Face* open_face;
    std::set<Dcel::Vertex*> vertices_pruned;
    std::set<Dcel::HalfEdge*> edges_pruned;

    // Clockwise and counter-clockwise neighbours
    std::vector<int> cw, ccw;

    // Find index in hull
    std::vector<int> inv;

    // First edge for every face
    // Note: the face is *right* from the edge
    std::map<int, Dcel::HalfEdge*> first_edge;

    std::vector<geometry::Point> sites;
    std::vector<geometry::Point> hull;

    Model* model;
    Dcel* dcel;

    const int delay_ms = 400;  // Slow the algorithm down for better visualization of steps
};