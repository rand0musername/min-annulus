#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include "model.h"

class Window {
   public:
    // Chooses what to show
    enum Mode { NOTHING, VORONOI, FP_VORONOI, CANDIDATES, ANNULUS };
    Window(Model* model);
    ~Window();

    // Draw every tick_ms
    void Draw(int tick_ms);

   private:
    void SaveToPhoto(const std::string& filename);

    Model* model;

    std::unique_ptr<sf::RenderWindow> window;
    sf::View view;
    sf::Font font;

    bool focus;  // Is the window in focus?
    std::vector<sf::Color> face_colors;
    std::vector<sf::Color> fp_face_colors;

    Mode mode;

    void DrawSweepLine(sf::Color color);
    void DrawSites(sf::Color color);
    void DrawFaces(Dcel* dcel, const std::vector<sf::Color>& colors);
    void DrawVertices(Dcel* dcel, sf::Color color);
    void DrawEdges(Dcel* dcel, sf::Color color);
    void DrawCandidates(sf::Color color);
    void DrawAnnulus(sf::Color color);
};
