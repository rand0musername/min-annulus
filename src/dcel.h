#pragma once
#include <vector>
#include "geometry.h"

class Dcel {
   public:
    ~Dcel();

    struct Vertex;
    struct Face;
    struct HalfEdge;

    // Prints the whole DCEL
    void Print();

    struct Vertex {
        geometry::Point point;
        HalfEdge* incident_halfedge;
        bool box;  // Is this a box vertex?

        Vertex(geometry::Point point) {
            this->point = point;
            incident_halfedge = nullptr;
            box = false;
        }
        Vertex(geometry::Point point, bool box) : Vertex(point) { this->box = box; }
    };

    struct Face {
        HalfEdge* outer_component;
        std::vector<HalfEdge*> inner_components;
        int site;  // Index of the site tied to this face
        Face(int site) {
            this->site = site;
            outer_component = nullptr;
        }
    };

    struct HalfEdge {
        Vertex* origin;
        HalfEdge* twin;
        Face* incident_face;
        HalfEdge* next;
        HalfEdge* prev;
        geometry::Line line;  // Line equation

        HalfEdge() {
            origin = nullptr;
            incident_face = nullptr;
            twin = next = prev = nullptr;
        }

        // Prints a half-edge
        void Print();
    };

    std::vector<Vertex*> vertices;
    std::vector<Face*> faces;
    std::vector<HalfEdge*> half_edges;
};