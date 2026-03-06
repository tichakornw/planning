#include "Vertex.h"
#include <cassert>

// Vertex Implementation
Vertex::Vertex(size_t vid) : vid(vid) {}

Vertex::~Vertex() {
    assert(in_edges.empty() && "in_edges not cleared");
    assert(out_edges.empty() && "out_edges not cleared");
}

void Vertex::addOutEdge(const std::shared_ptr<Edge> &edge) {
    out_edges.push_back(edge);
}

void Vertex::addInEdge(const std::shared_ptr<Edge> &edge) {
    in_edges.push_back(edge);
}

void Vertex::clear() {
    in_edges.clear();  // Clear incoming edge pointers
    out_edges.clear(); // Clear outgoing edge pointers
}

std::ostream &operator<<(std::ostream &os, const Vertex &v) {
    os << v.vid;
    return os;
}

// Edge Implementation
Edge::Edge(const std::shared_ptr<Vertex> &v1, const std::shared_ptr<Vertex> &v2, size_t eid)
    : eid(eid), from(v1), to(v2) {}

Edge::~Edge() {
    if (from || to) {
        std::cerr << "Edge destructor called without proper clear. "
                  << "from: " << (from ? from->vid : -1)
                  << ", to: " << (to ? to->vid : -1) << std::endl;
    }
    assert(!from && "from vertex not cleared");
    assert(!to && "to vertex not cleared");
}

void Edge::clear() {
    from.reset(); // Drop reference to source vertex
    to.reset();   // Drop reference to destination vertex
}

std::ostream &operator<<(std::ostream &os, const Edge &e) {
    os << "[" << e.from->vid << "," << e.to->vid << "]";
    return os;
}