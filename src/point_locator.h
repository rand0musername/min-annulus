#pragma once
#include <map>
#include <vector>
#include "dcel.h"
#include "geometry.h"

// Preprocesses a given DCEL for point location queries
// Uses vertical slabs
// TODO: trapezoidal maps
class PointLocator {
   public:
    // Preprocess a DCEL
    void LoadDcel(Dcel* dcel);

    // Answer a query: find a face that holds a given point
    int Locate(geometry::Point pt);

   private:
    // A single line that intersects a slab
    struct Info {
        geometry::Line line;
        int site_below;
        int site_above;
    };

    // Maps right_y to a vector of lines that intersect a slab
    std::map<double, std::vector<Info>> slabs;

    // If all edges are vertical it's a special case
    bool verticals = false;
    int vert_L, vert_R;
    double vert_thresh;
};
