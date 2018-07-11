#include "window.h"

#include <cassert>
#include <sstream>
#include "dcel.h"
#include "geometry.h"
#include "model.h"

Window::Window(Model* model) {
    mode = NOTHING;  // Initial mode
    this->model = model;

    // Create the window
    window = std::make_unique<sf::RenderWindow>(sf::VideoMode(800, 800), "min-annulus");

    // Find max/min site x/y
    double minx = model->GetPoint(0).x, maxx = model->GetPoint(0).x;
    double miny = model->GetPoint(0).y, maxy = model->GetPoint(0).y;
    for (geometry::Point pt : model->GetPoints()) {
        minx = std::min(minx, pt.x);
        maxx = std::max(maxx, pt.x);
        miny = std::min(miny, pt.y);
        maxy = std::max(maxy, pt.y);
    }

    // Calculate center and size
    // Note: we are drawing -y for y to flip the plane vertically
    //       (that way x axis goes upwards)
    double sz = std::max(maxx - minx, maxy - miny);
    double mid_x = (minx + maxx) / 2;
    double mid_y = (miny + maxy) / 2;
    view.setCenter(sf::Vector2f(mid_x, -mid_y));
    view.setSize(sf::Vector2f(sz * 1.5, sz * 1.5));
    window->setView(view);

    // Load font
    assert(font.loadFromFile("assets/Courier.ttf"));

    // We are initially focused
    focus = true;

    // Pick face colors once
    int n = model->GetNumSites();
    for (int i = 0; i < n; i++) {
        face_colors.push_back(sf::Color(rand() % 255, rand() % 255, rand() % 255, 140));
        fp_face_colors.push_back(sf::Color(rand() % 255, rand() % 255, rand() % 255, 140));
    }
}

Window::~Window() { window.reset(); }

void Window::Draw(int tick_ms) {
    // Move and zoom speeds
    double mv = 0.02 * view.getSize().x;
    double zm = 0.05;

    while (true) {
        // Event loop
        sf::Event event;
        while (window->pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window->close();
            }
            if (event.type == sf::Event::GainedFocus) {
                focus = true;
            }
            if (event.type == sf::Event::LostFocus) {
                focus = false;
            }
            if (event.type == sf::Event::KeyPressed) {
                // Save a photo or change mode
                if (event.key.code == sf::Keyboard::P) {
                    std::stringstream buffer;
                    buffer << "images/" << mode << ".png";
                    SaveToPhoto(buffer.str());
                    printf("Photo %s saved - all photos merged\n", buffer.str().c_str());
                    system("src/utils/merge.sh");
                } else if (event.key.code == sf::Keyboard::Num1) {
                    mode = NOTHING;
                } else if (event.key.code == sf::Keyboard::Num2) {
                    mode = VORONOI;
                } else if (event.key.code == sf::Keyboard::Num3) {
                    mode = FP_VORONOI;
                } else if (event.key.code == sf::Keyboard::Num4) {
                    mode = CANDIDATES;
                } else if (event.key.code == sf::Keyboard::Num5) {
                    mode = ANNULUS;
                }
            }
            if (event.type == sf::Event::MouseWheelMoved) {
                // Zoom
                if (event.mouseWheel.delta < 0)
                    view.zoom(std::pow(1 + zm, std::abs(event.mouseWheel.delta)));
                else
                    view.zoom(std::pow(1 - zm, std::abs(event.mouseWheel.delta)));
            }
        }

        // Keyboard controls
        sf::Text mouse_text;
        if (focus) {
            // Move
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::W)) {
                view.move(0, -mv);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)) {
                view.move(0, mv);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::A)) {
                view.move(-mv, 0);
            }
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)) {
                view.move(mv, 0);
            }
        }
        window->setView(view);

        // Clear the window
        window->clear(sf::Color::White);

        // Draw Voronoi
        if (mode == VORONOI || mode == CANDIDATES) {
            std::lock_guard<std::mutex> lock(*(model->GetMutex()));
            DrawFaces(model->GetVoronoiDcel(), face_colors);
            DrawVertices(model->GetVoronoiDcel(), sf::Color::Blue);
            DrawEdges(model->GetVoronoiDcel(), sf::Color::Blue);
            DrawSweepLine(sf::Color::Blue);
        }

        // Draw farthest-point Voronoi
        if (mode == FP_VORONOI || mode == CANDIDATES) {
            std::lock_guard<std::mutex> lock(*(model->GetMutex()));
            DrawFaces(model->GetFpVoronoiDcel(), fp_face_colors);
            DrawVertices(model->GetFpVoronoiDcel(), sf::Color::Red);
            DrawEdges(model->GetFpVoronoiDcel(), sf::Color::Red);
        }

        // Draw candidates
        if (mode == CANDIDATES) {
            std::lock_guard<std::mutex> lock(*(model->GetMutex()));
            DrawCandidates(sf::Color::White);
        }

        // Draw winning annulus
        if (mode == ANNULUS) {
            std::lock_guard<std::mutex> lock(*(model->GetMutex()));
            DrawAnnulus(sf::Color(229, 187, 51));
        }

        // Draw original sites
        DrawSites(sf::Color::Black);

        window->display();

        // Wait for the next tick
        std::this_thread::sleep_for(std::chrono::milliseconds(tick_ms));
    }
}

void Window::SaveToPhoto(const std::string& filename) {
    sf::Vector2u windowSize = window->getSize();
    sf::Texture texture;
    texture.create(windowSize.x, windowSize.y);
    texture.update(*window);
    sf::Image screenshot = texture.copyToImage();
    screenshot.saveToFile(filename);
}

void Window::DrawFaces(Dcel* dcel, const std::vector<sf::Color>& colors) {
    // Draw faces
    int idx = 0;
    for (Dcel::HalfEdge* he : dcel->faces.back()->inner_components) {
        Dcel::HalfEdge* curr = he;
        sf::ConvexShape poly;

        // Find the number of polygon vertices
        int nb = 0;
        do {
            nb++;
            curr = curr->next;
        } while (curr != he);
        poly.setPointCount(nb);

        // Get polygon points
        curr = he;
        nb = 0;
        do {
            poly.setPoint(nb, sf::Vector2f(curr->origin->point.x, -curr->origin->point.y));
            nb++;
            curr = curr->next;
        } while (curr != he);

        // Draw
        poly.setFillColor(colors[idx++]);
        window->draw(poly);
    }
}

void Window::DrawSweepLine(sf::Color color) {
    sf::Vertex edge[] = {
        sf::Vertex(sf::Vector2f(view.getCenter().x - view.getSize().x / 2, -model->GetSweepY()), color),
        sf::Vertex(sf::Vector2f(view.getCenter().x + view.getSize().x / 2, -model->GetSweepY()), color),
    };
    window->draw(edge, 2, sf::Lines);
}

void Window::DrawSites(sf::Color color) {
    int n = model->GetNumSites();
    for (int i = 0; i < n; i++) {
        // Draw a site
        double radius = 3;
        sf::CircleShape vertex(radius);
        vertex.setPosition(model->GetPoint(i).x - radius, -model->GetPoint(i).y - radius);
        vertex.setFillColor(color);
        window->draw(vertex);
    }
}

void Window::DrawVertices(Dcel* dcel, sf::Color color) {
    int n = dcel->vertices.size();
    for (int i = 0; i < n; i++) {
        if (mode == CANDIDATES && dcel->vertices[i]->box) continue;
        // Draw a DCEL vertex
        double radius = 2;
        sf::CircleShape vertex(radius);
        vertex.setPosition(dcel->vertices[i]->point.x - radius, -dcel->vertices[i]->point.y - radius);
        vertex.setFillColor(color);
        window->draw(vertex);
    }
}

void Window::DrawEdges(Dcel* dcel, sf::Color color) {
    int n = dcel->half_edges.size();
    for (int i = 0; i < n; i++) {
        Dcel::HalfEdge* he = dcel->half_edges[i];
        Dcel::HalfEdge* twin = he->twin;
        if (he->origin != nullptr && twin->origin != nullptr) {
            // Draw a DCEL half-edge
            sf::Vertex edge[] = {
                sf::Vertex(sf::Vector2f(he->origin->point.x, -he->origin->point.y), color),
                sf::Vertex(sf::Vector2f(twin->origin->point.x, -twin->origin->point.y), color),
            };
            window->draw(edge, 2, sf::Lines);
        }
    }
}

void Window::DrawCandidates(sf::Color color) {
    for (auto candidate : model->GetCandidates()) {
        // Draw a DCEL vertex
        double radius = 2;
        sf::CircleShape vertex(radius);
        vertex.setPosition(candidate.center.x - radius, -candidate.center.y - radius);
        vertex.setFillColor(color);
        window->draw(vertex);
    }
}

void Window::DrawAnnulus(sf::Color color) {
    geometry::Annulus* annulus = model->GetAnnulus();

    if (annulus->r_outer == -1) return;

    // Outer circle
    sf::CircleShape outer(annulus->r_outer);
    outer.setOrigin(sf::Vector2f(annulus->r_outer, annulus->r_outer));
    outer.setPosition(sf::Vector2f(annulus->center.x, -annulus->center.y));
    outer.setOutlineThickness(1);
    outer.setOutlineColor(color);
    outer.setFillColor(sf::Color::Transparent);
    window->draw(outer);

    // Inner circle
    sf::CircleShape inner(annulus->r_inner);
    inner.setOrigin(sf::Vector2f(annulus->r_inner, annulus->r_inner));
    inner.setPosition(sf::Vector2f(annulus->center.x, -annulus->center.y));
    inner.setOutlineThickness(1);
    inner.setOutlineColor(color);
    inner.setFillColor(sf::Color::Transparent);
    window->draw(inner);

    // Draw
    double radius = 3;
    sf::CircleShape vertex(radius);
    vertex.setPosition(annulus->center.x - radius, -annulus->center.y - radius);
    vertex.setFillColor(color);
    window->draw(vertex);
}