#include "beach_line.h"

TreeNode::TreeNode(bool leaf) {
    this->leaf = leaf;
    this->id = rand();
    left = right = parent = nullptr;
}

LeafNode::LeafNode(int site) : TreeNode(true) {
    this->site = site;
    this->circle_event = nullptr;
}

InternalNode::InternalNode(std::pair<int, int> sites, Dcel::HalfEdge* half_edge) : TreeNode(false) {
    this->sites = sites;
    this->half_edge = half_edge;
}

TreeNode* BeachLine::GetRoot() { return root; }

LeafNode* BeachLine::FindArcAbove(double x, double sw_y) {
    if (root == nullptr) {
        return nullptr;
    }
    TreeNode* curr = root;

    // At each tree node go left or right based on the intersection x-coordinate
    while (!(curr->IsLeaf())) {
        InternalNode* curr_inter = static_cast<InternalNode*>(curr);
        geometry::Point inter = geometry::FindParabolaIntersection(sites[curr_inter->GetSites().first],
                                                                   sites[curr_inter->GetSites().second], sw_y);
        if (inter.x < x) {
            curr = curr_inter->GetRight();
        } else {
            curr = curr_inter->GetLeft();
        }
    }
    return static_cast<LeafNode*>(curr);
}

void BeachLine::SetRoot(TreeNode* node) { root = node; }

InternalNode* BeachLine::FindPredLca(LeafNode* leaf) {
    TreeNode* curr = leaf;
    if (curr == nullptr) {
        return nullptr;
    }
    // While curr is the left child keep climbing
    while (curr->GetParent() != nullptr && curr->GetParent()->GetLeft() == curr) {
        curr = curr->GetParent();
    }
    return static_cast<InternalNode*>(curr->GetParent());
}

LeafNode* BeachLine::FindPred(LeafNode* leaf) {
    TreeNode* curr = FindPredLca(leaf);
    if (curr == nullptr) {
        return nullptr;
    }
    // One left, then right until the leaf
    curr = curr->GetLeft();
    while (curr->GetRight() != nullptr) {
        curr = curr->GetRight();
    }
    return static_cast<LeafNode*>(curr);
}

InternalNode* BeachLine::FindSuccLca(LeafNode* leaf) {
    TreeNode* curr = leaf;
    if (curr == nullptr) {
        return nullptr;
    }
    // While curr is the right child keep climbing
    while (curr->GetParent() != nullptr && curr->GetParent()->GetRight() == curr) {
        curr = curr->GetParent();
    }
    return static_cast<InternalNode*>(curr->GetParent());
}

LeafNode* BeachLine::FindSucc(LeafNode* leaf) {
    TreeNode* curr = FindSuccLca(leaf);
    if (curr == nullptr) {
        return nullptr;
    }
    // One right, then lef until the leaf
    curr = curr->GetRight();
    while (curr->GetLeft() != nullptr) {
        curr = curr->GetLeft();
    }
    return static_cast<LeafNode*>(curr);
}

void BeachLine::InitialInsert(int site, Dcel::HalfEdge* he) {
    int leftest_site;

    if (root->IsLeaf()) {
        leftest_site = static_cast<InternalNode*>(root)->GetSites().first;
    } else {
        leftest_site = static_cast<LeafNode*>(root)->GetSite();
    }

    InternalNode* new_root = new InternalNode({site, leftest_site}, he);
    LeafNode* leaf = new LeafNode(site);
    new_root->SetLeft(leaf);
    leaf->SetParent(new_root);
    new_root->SetRight(root);
    root->SetParent(new_root);
    root = new_root;
}

LeafNode* BeachLine::Insert(LeafNode* curr, int site, Dcel::HalfEdge* upper, Dcel::HalfEdge* lower) {
    int other = curr->GetSite();
    TreeNode* parent = curr->GetParent();
    char side;
    if (parent != nullptr && parent->GetLeft() == curr) {
        side = 'l';
    } else {
        side = 'r';
    }
    if (curr->GetCircleEvent() != nullptr) {
        curr->GetCircleEvent()->SetArc(nullptr);
    }
    delete curr;

    // Add 5 new nodes: 3 leaves and 2 internal intersections
    LeafNode* leaf1 = new LeafNode(other);
    LeafNode* leaf2 = new LeafNode(site);
    LeafNode* leaf3 = new LeafNode(other);
    InternalNode* internal1 = new InternalNode({other, site}, upper);
    InternalNode* internal2 = new InternalNode({site, other}, lower);
    if (parent == nullptr) {
        root = internal1;
    } else {
        if (side == 'l') {
            parent->SetLeft(internal1);
        } else {
            parent->SetRight(internal1);
        }
        internal1->SetParent(parent);
    }

    // Link nodes
    internal1->SetLeft(leaf1);
    leaf1->SetParent(internal1);
    internal1->SetRight(internal2);
    internal2->SetParent(internal1);
    internal2->SetLeft(leaf2);
    leaf2->SetParent(internal2);
    internal2->SetRight(leaf3);
    leaf3->SetParent(internal2);
    return leaf2;
}

std::pair<Dcel::HalfEdge*, Dcel::HalfEdge*> BeachLine::Delete(LeafNode* arc, Dcel::HalfEdge* nw) {
    // Find pred/succ
    LeafNode* pred = FindPred(arc);
    LeafNode* succ = FindSucc(arc);
    InternalNode* parent = static_cast<InternalNode*>(arc->GetParent());  // LCA 1
    char side = (parent->GetLeft() == arc) ? 'l' : 'r';
    TreeNode* sibling = (side == 'l') ? parent->GetRight() : parent->GetLeft();

    // Find LCA 2
    InternalNode* down = parent;
    InternalNode* up = static_cast<InternalNode*>(parent->GetParent());
    while ((side == 'l' && up->GetLeft() == down) || (side == 'r' && up->GetRight() == down)) {
        down = up;
        up = static_cast<InternalNode*>(up->GetParent());
    }
    InternalNode* other_lca = up;  // LCA 2

    // Save half edges that got merged
    std::pair<Dcel::HalfEdge*, Dcel::HalfEdge*> ret;
    if (side == 'r') {
        ret.first = parent->GetHalfEdge();
        ret.second = other_lca->GetHalfEdge();
    }
    if (side == 'l') {
        ret.first = other_lca->GetHalfEdge();
        ret.second = parent->GetHalfEdge();
    }
    other_lca->SetHalfEdge(nw);

    // Delete
    InternalNode* grandpa = static_cast<InternalNode*>(parent->GetParent());
    char parent_side = (grandpa->GetLeft() == parent) ? 'l' : 'r';

    auto old_sites = other_lca->GetSites();
    if (side == 'l') {
        other_lca->SetSites({old_sites.first, succ->GetSite()});
    } else {
        other_lca->SetSites({pred->GetSite(), old_sites.second});
    }

    // This block works for both values of side
    if (parent_side == 'l') {
        grandpa->SetLeft(sibling);
    } else {
        grandpa->SetRight(sibling);
    }
    sibling->SetParent(grandpa);
    delete arc;
    delete parent;

    // return
    return ret;
}

void BeachLine::PrintToDot(TreeNode* node, std::ofstream& dot_file) {
    if (node == nullptr) {
        return;
    }

    // Node
    int id = node->GetId();
    dot_file << id << "[label=\"leaf(" << node->IsLeaf() << ")";
    if (node->IsLeaf()) {
        LeafNode* curr = nullptr;
        curr = static_cast<LeafNode*>(node);
        dot_file << " site: (" << curr->GetSite() << ")\"];\n";
    } else {
        InternalNode* curr = nullptr;
        curr = static_cast<InternalNode*>(node);
        dot_file << " sites:(" << curr->GetSites().first << "," << curr->GetSites().second << ")\"];\n";
    }

    // Right child
    if (node->GetLeft() != nullptr) {
        dot_file << id << " -> " << node->GetLeft()->GetId() << "[label=\"L\"]\n";
        PrintToDot(node->GetLeft(), dot_file);
    } else {
        dot_file << "nullptr_l_" << id << "[shape=point];\n";
        dot_file << id << " -> "
                 << "nullptr_l_" << id << "[label=\"L\"]\n";
    }

    // Left child
    if (node->GetRight() != nullptr) {
        dot_file << id << " -> " << node->GetRight()->GetId() << "[label=\"R\"]\n";
        PrintToDot(node->GetRight(), dot_file);
    } else {
        dot_file << "nullptr_r_" << id << "[shape=point];\n";
        dot_file << id << " -> "
                 << "nullptr_r_" << id << "[label=\"R\"]\n";
    }

    // Parent
    if (node->GetParent() != nullptr)
        dot_file << id << " -> " << node->GetParent()->GetId() << "[color=red style=dashed]\n";
}

void BeachLine::Draw(std::string name) {
    std::ofstream dot_file;
    dot_file.open(name + ".dot");
    dot_file << "digraph{\n";
    PrintToDot(root, dot_file);
    dot_file << "}\n";
    dot_file.close();
    std::string command = "dot -Tpng " + name + ".dot -o " + name + ".png";
    system(command.c_str());
}

void BeachLine::Cleanup(TreeNode* curr) {
    if (curr == nullptr) {
        return;
    }
    Cleanup(curr->GetLeft());
    Cleanup(curr->GetRight());
    delete curr;
}

BeachLine::~BeachLine() { Cleanup(root); }

LeafNode* BeachLine::GetFirstLeaf() {
    if (root == nullptr) {
        return nullptr;
    }
    TreeNode* curr = root;
    // Go left while you can
    while (!(curr->IsLeaf())) {
        curr = curr->GetLeft();
    }
    return static_cast<LeafNode*>(curr);
}

void BeachLine::SetOrientations(double sw_y) {
    if (root != nullptr && !root->IsLeaf()) {
        SetOrientation(static_cast<InternalNode*>(root), sw_y);
    }
}

void BeachLine::SetOrientation(InternalNode* curr, double sw_y) {
    Dcel::HalfEdge* half_edge = curr->GetHalfEdge();
    Dcel::HalfEdge* edge = (half_edge->origin == nullptr) ? half_edge->twin : half_edge;

    // 'edge' is the one with origin, find near and far point
    geometry::Point near = edge->origin->point;
    geometry::Point far =
        geometry::FindParabolaIntersection(sites[curr->GetSites().first], sites[curr->GetSites().second], sw_y);

    // Compare near and far to set the orientation
    if (edge->line.vertical) {
        if (near.y < far.y) {
            edge->line.dir = 'u';
        } else {
            edge->line.dir = 'd';
        }
    } else {
        if (near.x < far.x) {
            edge->line.dir = 'r';
        } else if (near.x > far.x) {
            edge->line.dir = 'l';
        }
    }

    // Recurse
    if (curr->GetLeft() != nullptr && !curr->GetLeft()->IsLeaf()) {
        SetOrientation(static_cast<InternalNode*>(curr->GetLeft()), sw_y);
    }
    if (curr->GetRight() != nullptr && !curr->GetRight()->IsLeaf()) {
        SetOrientation(static_cast<InternalNode*>(curr->GetRight()), sw_y);
    }
}
