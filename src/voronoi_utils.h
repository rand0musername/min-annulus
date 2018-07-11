#pragma once
#include <vector>
#include "dcel.h"
#include "geometry.h"

namespace voronoi_utils {

// Adds a bounding box around a diagram given in a DCEL, open_face is the unbounded face
Dcel::HalfEdge* AddBox(const std::vector<geometry::Point>& sites, Dcel::Face* open_face, Dcel* dcel);

}  // namespace voronoi_utils