#ifndef _GRAPH_H
#define _GRAPH_H

#include <algorithm>
#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <memory>
#include <iostream>
#include <cassert>
#include <stdexcept>

#include "Vertex.h"

class Graph {
  public:
    Graph() = default;
    virtual ~Graph();

    // Rule of Five (Copy and Move semantics)
    Graph(const Graph &other);
    Graph &operator=(const Graph &other);
    Graph(Graph &&other) noexcept;
    Graph &operator=(Graph &&other) noexcept;

    // Vertex operations
    size_t addVertex();
    void addVertex(size_t vid);
    void removeVertex(size_t vertex_id);
    [[nodiscard]] const std::shared_ptr<Vertex> getVertex(size_t vid) const;
    [[nodiscard]] const std::unordered_map<size_t, std::shared_ptr<Vertex>> &getVertices() const;
    [[nodiscard]] size_t getNumVertices() const { return vertices.size(); }

    // Edge operations
    size_t addEdge(size_t from, size_t to);
    void addEdge(size_t from, size_t to, size_t eid, bool allow_duplicate = true);
    void removeEdge(const std::shared_ptr<Edge> &edge);
    [[nodiscard]] const std::shared_ptr<Edge> getEdge(size_t eid) const;
    [[nodiscard]] const std::vector<std::shared_ptr<Edge>> &getEdges() const;
    [[nodiscard]] size_t getNumEdges() const { return edges.size(); }

    // Utility operations
    void clear();
    [[nodiscard]] std::unordered_set<size_t> findSuccessors(size_t start_id) const;
    std::vector<size_t> topologicalSort() const;
    [[nodiscard]] std::vector<std::shared_ptr<Edge>> getPath(size_t from, size_t to) const;
    
    // Graph property checks
    bool isDAG() const;
    bool isTotallyOrdered() const;
    [[nodiscard]] bool is_consistent() const;
    virtual void display() const;

  protected:
    std::unordered_map<size_t, std::shared_ptr<Vertex>> vertices;
    std::vector<std::shared_ptr<Edge>> edges;

    bool addEdgeToGraph(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2,
                        std::shared_ptr<Edge> edge, bool allow_duplicate);
    void removeEdgeFromVertices(const std::shared_ptr<Edge> &edge);

    // Shared DFS utility
    bool dfsVisit(const std::shared_ptr<Vertex> &vertex,
                  std::unordered_map<std::shared_ptr<Vertex>, bool> &visited,
                  std::unordered_map<std::shared_ptr<Vertex>, bool> &on_stack,
                  std::unordered_map<size_t, std::shared_ptr<Edge>> &parent_edge,
                  std::stack<size_t> *topo_stack = nullptr,
                  bool *cycle_detected = nullptr) const;
};

#endif