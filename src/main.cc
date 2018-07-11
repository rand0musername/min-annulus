#include <SFML/Graphics.hpp>
#include <fstream>
#include <iostream>
#include "annulus_finder.h"
#include "fp_voronoi.h"
#include "model.h"
#include "voronoi.h"
#include "window.h"

using namespace std;

// To run: ./main <testcase_path>
int main(int argc, char* argv[]) {
    srand(time(NULL));
    printf("Move:ASDF | Zoom:Scroll | Change Mode:Num1-Num5 | Save Photo:P\n");
    printf("Modes: (1) Nothing (2) Voronoi (3) FP Voronoi (4) Candidates (5) Annulus\n");

    // Grab command-line arguments
    if (argc != 2) {
        std::cout << "Error: there should be exactly 1 command-line argument." << endl;
        return 0;
    }

    // Load the testcase
    vector<geometry::Point> points;
    ifstream in_file(argv[1]);
    int n;
    in_file >> n;
    for (int i = 0; i < n; i++) {
        double x, y;
        in_file >> x >> y;
        points.push_back({x, y, i});
    }

    // Start
    printf("Starting!\n");
    Model model(points);

    // Compute a voronoi diagram in a new thread
    Voronoi voronoi(&model);
    std::future<void> v_fut = voronoi.ComputeDiagram();

    // Compute a farthest-point Voronoi diagram in a new thread
    FarthestPointVoronoi fp_voronoi(&model);
    std::future<void> fpv_fut = fp_voronoi.ComputeDiagram();

    // Combine two diagrams to find the solution in a new thread
    AnnulusFinder annulus_finder(&v_fut, &fpv_fut, &model);
    std::future<void> ann_fut = annulus_finder.FindAnnulus();

    // GUI: draw every 10ms (main thread)
    Window window(&model);
    window.Draw(10);
    return 0;
}