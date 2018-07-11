#pragma once
#include <fstream>
#include "dcel.h"
#include "event.h"
#include "geometry.h"

class CircleEvent;

class TreeNode {
   public:
    TreeNode(bool leaf);

    void SetLeft(TreeNode* left) { this->left = left; }
    void SetRight(TreeNode* right) { this->right = right; }
    void SetParent(TreeNode* parent) { this->parent = parent; }
    TreeNode* GetLeft() { return left; }
    TreeNode* GetRight() { return right; }
    TreeNode* GetParent() { return parent; }
    int GetId() { return id; }
    bool IsLeaf() { return leaf; }

   private:
    TreeNode* left;
    TreeNode* right;
    TreeNode* parent;
    int id;  // For graphviz
    bool leaf;
};

class LeafNode : public TreeNode {
   public:
    LeafNode(int site);

    void SetCircleEvent(CircleEvent* circle_event) { this->circle_event = circle_event; }
    int GetSite() { return site; }
    CircleEvent* GetCircleEvent() { return circle_event; }

   private:
    int site;
    CircleEvent* circle_event;
};

class InternalNode : public TreeNode {
   public:
    InternalNode(std::pair<int, int> sites, Dcel::HalfEdge* half_edge);

    std::pair<int, int> GetSites() { return sites; }
    void SetSites(std::pair<int, int> sites) { this->sites = sites; }
    Dcel::HalfEdge* GetHalfEdge() { return half_edge; }
    void SetHalfEdge(Dcel::HalfEdge* half_edge) { this->half_edge = half_edge; }

   private:
    std::pair<int, int> sites;
    Dcel::HalfEdge* half_edge;
};

// A BST that holds information about the beach line
// Internal nodes represent parabola intersections
// Leaf nodes represent arcs of the beach line
// TODO: add balancing operations (integrate AVL ops?)
class BeachLine {
   public:
    ~BeachLine();
    void SetSites(const std::vector<geometry::Point>& sites) { this->sites = sites; }
    TreeNode* GetRoot();
    LeafNode* GetFirstLeaf();

    // Finds a beach line arc above a given point
    LeafNode* FindArcAbove(double x, double sw_y);

    void SetRoot(TreeNode* node);

    // Finds the predecessor
    LeafNode* FindPred(LeafNode* leaf);

    // Finds the successor
    LeafNode* FindSucc(LeafNode* leaf);

    // Finds LCA(leaf, predecessor)
    InternalNode* FindPredLca(LeafNode* leaf);

    // Finds LCA(leaf, successor)
    InternalNode* FindSuccLca(LeafNode* leaf);

    // First insertion is treated differently
    void InitialInsert(int site, Dcel::HalfEdge* he);

    // Regular insertion
    LeafNode* Insert(LeafNode* curr, int site, Dcel::HalfEdge* upper, Dcel::HalfEdge* lower);

    // Deletion
    std::pair<Dcel::HalfEdge*, Dcel::HalfEdge*> Delete(LeafNode* arc, Dcel::HalfEdge* down);

    // Sets half-edge orientations
    void SetOrientations(double sw_y);

    // Prints for GraphViz
    void PrintToDot(TreeNode* curr, std::ofstream& dot_file);
    void Draw(std::string name);

   private:
    std::vector<geometry::Point> sites;
    TreeNode* root = nullptr;

    // Sets half-edge orientation
    void SetOrientation(InternalNode* curr, double sw_y);

    // Deletes everything
    void Cleanup(TreeNode* curr);
};