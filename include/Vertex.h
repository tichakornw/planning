#ifndef _VERTEX_H
#define _VERTEX_H

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

    void addOutEdge(const std::shared_ptr<Edge> &edge) {
        out_edges.push_back(edge);
    }

    void addInEdge(const std::shared_ptr<Edge> &edge) {
        in_edges.push_back(edge);
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
    virtual ~Edge() {}

    friend std::ostream &operator<<(std::ostream &os, const Edge &e) {
        os << "[" << e.from->vid << "," << e.to->vid << "]";
        return os;
    }
};

#endif
