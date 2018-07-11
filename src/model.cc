#include "model.h"

Model::Model(const std::vector<geometry::Point>& points) {
    mutex = new std::mutex();

    this->points = new std::vector<geometry::Point>(points);
    InitVoronoiSweepLine();
    voronoi_dcel = new Dcel();
    fp_voronoi_dcel = new Dcel();

    annulus = new geometry::Annulus();
    ann_candidates = new std::vector<geometry::Annulus>();
}

void Model::InitVoronoiSweepLine() {
    sweep_y = (*points)[0].y + 10;
    for (auto pt : *points) {
        sweep_y = std::max(sweep_y, pt.y + 10);
    }
}