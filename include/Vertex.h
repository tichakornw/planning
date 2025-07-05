#ifndef _VERTEX_H
#define _VERTEX_H

#include <cassert>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

// Forward declaration of Edge to use it in Vertex
class Edge;

class Vertex {
  public:
    size_t vid;
    std::vector<std::shared_ptr<Edge>> out_edges;
    std::vector<std::shared_ptr<Edge>> in_edges;

    Vertex(size_t vid) : vid(vid) {}

    ~Vertex() {
        assert(in_edges.empty() && "in_edges not cleared");
        assert(out_edges.empty() && "out_edges not cleared");
    }

    void addOutEdge(const std::shared_ptr<Edge> &edge) {
        out_edges.push_back(edge);
    }

    void addInEdge(const std::shared_ptr<Edge> &edge) {
        in_edges.push_back(edge);
    }

    void clear() {
        in_edges.clear();  // Clear incoming edge pointers
        out_edges.clear(); // Clear outgoing edge pointers
    }

    friend std::ostream &operator<<(std::ostream &os, const Vertex &v) {
        os << v.vid;
        return os;
    }
};

class Edge {
  public:
    size_t eid;
    std::shared_ptr<Vertex> from;
    std::shared_ptr<Vertex> to;

    Edge(const std::shared_ptr<Vertex> &v1, const std::shared_ptr<Vertex> &v2,
         size_t eid)
        : eid(eid), from(v1), to(v2) {}

    // Add a virtual destructor to make Edge polymorphic
    virtual ~Edge() {
        if (from || to) {
            std::cerr << "Edge destructor called without proper clear. "
                      << "from: " << (from ? from->vid : -1)
                      << ", to: " << (to ? to->vid : -1) << std::endl;
        }
        assert(!from && "from vertex not cleared");
        assert(!to && "to vertex not cleared");
    }

    void clear() {
        from.reset(); // Drop reference to source vertex
        to.reset();   // Drop reference to destination vertex
    }

    friend std::ostream &operator<<(std::ostream &os, const Edge &e) {
        os << "[" << e.from->vid << "," << e.to->vid << "]";
        return os;
    }
};

#endif
