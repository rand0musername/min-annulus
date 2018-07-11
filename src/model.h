#pragma once
#include <algorithm>
#include <future>
#include <mutex>
#include <vector>
#include "dcel.h"
#include "geometry.h"

class Model {
   public:
    Model(const std::vector<geometry::Point>& points);

    double GetSweepY() { return sweep_y; }

    std::mutex* GetMutex() { return mutex; }

    Dcel* GetVoronoiDcel() { return voronoi_dcel; }

    Dcel* GetFpVoronoiDcel() { return fp_voronoi_dcel; }

    geometry::Point GetPoint(int idx) { return (*points)[idx]; }

    std::vector<geometry::Point> GetPoints() { return *points; }

    geometry::Point GetHullPoint(int idx) { return hull[idx]; }

    std::vector<geometry::Point> GetHull() { return hull; }

    void SetSweepY(double y) { sweep_y = y; }

    void SetHull(const std::vector<geometry::Point>& hull) { this->hull = hull; }

    void AddAnnCandidate(const geometry::Annulus& ann) { ann_candidates->push_back(ann); }

    std::vector<geometry::Annulus> GetCandidates() { return *ann_candidates; }

    // Sort by width and select best
    void FindBestAnnulus() {
        std::sort(ann_candidates->begin(), ann_candidates->end(),
                  [](const geometry::Annulus& a, const geometry::Annulus& b) {
                      return a.r_outer - a.r_inner < b.r_outer - b.r_inner;
                  });
        *annulus = (*ann_candidates)[0];
    }

    geometry::Annulus* GetAnnulus() { return annulus; }

    int GetNumSites() { return points->size(); }

   private:
    void InitVoronoiSweepLine();

    double sweep_y;  // For Voronoi
    std::mutex* mutex;
    Dcel* voronoi_dcel;
    Dcel* fp_voronoi_dcel;
    std::vector<geometry::Point>* points;

    geometry::Annulus* annulus;
    std::vector<geometry::Annulus>* ann_candidates;

    std::vector<geometry::Point> hull;  // For FP Voronoi, before shuffle
};