#ifndef _VERTEX_H
#define _VERTEX_H

#include <iostream>
#include <memory>
#include <vector>

// Forward declaration of Edge to use it in Vertex
class Edge;

class Vertex {
  public:
    size_t vid;
    std::vector<std::shared_ptr<Edge>> out_edges;
    std::vector<std::shared_ptr<Edge>> in_edges;

    Vertex(size_t vid);
    virtual ~Vertex();

    void addOutEdge(const std::shared_ptr<Edge> &edge);
    void addInEdge(const std::shared_ptr<Edge> &edge);
    void clear();

    friend std::ostream &operator<<(std::ostream &os, const Vertex &v);
};

class Edge {
  public:
    size_t eid;
    std::shared_ptr<Vertex> from;
    std::shared_ptr<Vertex> to;

    Edge(const std::shared_ptr<Vertex> &v1, const std::shared_ptr<Vertex> &v2, size_t eid);
    virtual ~Edge();

    void clear();

    friend std::ostream &operator<<(std::ostream &os, const Edge &e);
};

#endif