#pragma once
#include <future>
#include "model.h"
#include "point_locator.h"

class AnnulusFinder {
   public:
    AnnulusFinder(std::future<void>* fut1, std::future<void>* fut2, Model* model);

    // Finds the winning annulus in a new thread
    std::future<void> FindAnnulus();

   private:
    // Merges farthest-point Voronoi DCEL and Voronoi DCEL and finds the best annulus
    void MergeAndFind();

    // Generates annulus candidates
    void GenerateCandidates();

    // Futures from Voronoi threads
    std::future<void>* fut1;
    std::future<void>* fut2;

    // Point locators for both diagrams
    PointLocator voronoi_pl, fp_voronoi_pl;

    Model* model;
};
