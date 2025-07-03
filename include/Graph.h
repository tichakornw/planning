#ifndef _GRAPH_H
#define _GRAPH_H

#include <queue>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Vertex.h"

class Graph {
  public:
    // Add a vertex to the graph
    void addVertex(size_t vid) {
        if (vertices.find(vid) == vertices.end()) {
            vertices[vid] = std::make_shared<Vertex>(vid);
        }
    }

    // Add an edge to the graph
    void addEdge(size_t from, size_t to, size_t eid) {
        auto v1 = getVertex(from);
        auto v2 = getVertex(to);

        if (v1 && v2) {
            auto edge = std::make_shared<Edge>(v1, v2, eid);
            addEdgeToGraph(v1, v2, edge);
        }
    }

    void RemoveEdge(const std::shared_ptr<Edge>& edge) {
        // Remove from top-level edges
        edges.erase(std::remove(edges.begin(), edges.end(), edge), edges.end());
        RemoveEdgeFromVertices(edge);
    }


    const std::shared_ptr<Vertex> getVertex(size_t vid) const {
        auto it = vertices.find(vid);
        return it != vertices.end() ? it->second : nullptr;
    }

    const std::vector<std::shared_ptr<Edge>>& getEdges() const {
        return edges;
    }


    size_t getNumVertices() const { return vertices.size(); }

    std::unordered_set<size_t> findSuccessors(size_t start_id) const {
        std::unordered_set<size_t> successors;
        std::queue<size_t> to_visit;

        auto it = vertices.find(start_id);
        if (it == vertices.end()) {
            // Start vertex not found
            return successors;
        }

        to_visit.push(start_id);

        while (!to_visit.empty()) {
            size_t current_id = to_visit.front();
            to_visit.pop();

            auto current_vertex = vertices.at(current_id);
            for (const auto &edge : current_vertex->out_edges) {
                size_t neighbor_id = edge->to->vid;
                if (successors.find(neighbor_id) == successors.end()) {
                    successors.insert(neighbor_id);
                    to_visit.push(neighbor_id);
                }
            }
        }

        return successors;
    }

    // Perform topological sort using DFS
    std::vector<size_t> topologicalSort() const {
        std::vector<size_t> result;
        std::unordered_map<std::shared_ptr<Vertex>, bool> visited;
        std::stack<size_t> stack;

        // Mark all vertices as not visited
        for (const auto &pair : vertices) {
            visited[pair.second] = false;
        }

        // Perform DFS and push vertices to stack in topological order
        for (const auto &pair : vertices) {
            if (!visited[pair.second]) {
                topologicalSortUtil(pair.second, visited, stack);
            }
        }

        // Populate result from stack
        while (!stack.empty()) {
            result.push_back(stack.top());
            stack.pop();
        }

        return result;
    }

    // Display the graph
    virtual void display() const {
        for (const auto &pair : vertices) {
            std::cout << "  " << pair.first << ": [";
            for (size_t i = 0; i < pair.second->out_edges.size(); ++i) {
                auto edge = pair.second->out_edges[i];
                std::cout << edge->to->vid;
                if (i != pair.second->out_edges.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
    }

    // Check that the edges and those contained in vertices are consistent
    bool is_consistent() const {
        // Check that all edges in graph.edges are also in from->out_edges and to->in_edges
        for (const auto& edge : edges) {
            auto from = edge->from;
            auto to = edge->to;

            if (!from || !to) {
                std::cerr << "Edge has null endpoint.\n";
                return false;
            }

            // Check if edge is in from->out_edges
            const auto& out_edges = from->out_edges;
            if (std::find(out_edges.begin(), out_edges.end(), edge) == out_edges.end()) {
                std::cerr << "Edge from " << from->vid << " to " << to->vid
                          << " missing from from->out_edges.\n";
                return false;
            }

            // Check if edge is in to->in_edges
            const auto& in_edges = to->in_edges;
            if (std::find(in_edges.begin(), in_edges.end(), edge) == in_edges.end()) {
                std::cerr << "Edge from " << from->vid << " to " << to->vid
                          << " missing from to->in_edges.\n";
                return false;
            }
        }

        // Check that all vertex->in_edges and out_edges exist in graph.edges
        for (const auto& [vid, vertex] : vertices) {
            for (const auto& edge : vertex->out_edges) {
                if (std::find(edges.begin(), edges.end(), edge) == edges.end()) {
                    std::cerr << "Edge in vertex " << vid << " out_edges not found in graph.edges.\n";
                    return false;
                }
            }

            for (const auto& edge : vertex->in_edges) {
                if (std::find(edges.begin(), edges.end(), edge) == edges.end()) {
                    std::cerr << "Edge in vertex " << vid << " in_edges not found in graph.edges.\n";
                    return false;
                }
            }
        }
        return true;
    }

  protected:
    std::unordered_map<size_t, std::shared_ptr<Vertex>> vertices;
    std::vector<std::shared_ptr<Edge>> edges;

    void addEdgeToGraph(std::shared_ptr<Vertex> v1, std::shared_ptr<Vertex> v2,
                        std::shared_ptr<Edge> edge) {
        for (const auto &e : v1->out_edges) {
            if (e->to == v2) {
                return;
            }
        }
        v1->addOutEdge(edge);
        v2->addInEdge(edge);
        edges.push_back(edge);
    }

    void RemoveEdgeFromVertices(const std::shared_ptr<Edge>& edge) {
        auto& out_vec = edge->from->out_edges;
        out_vec.erase(std::remove(out_vec.begin(), out_vec.end(), edge), out_vec.end());

        auto& in_vec = edge->to->in_edges;
        in_vec.erase(std::remove(in_vec.begin(), in_vec.end(), edge), in_vec.end());
    }

    void topologicalSortUtil(
        const std::shared_ptr<Vertex> &vertex,
        std::unordered_map<std::shared_ptr<Vertex>, bool> &visited,
        std::stack<size_t> &stack) const {
        visited[vertex] = true;

        for (const auto &edge : vertex->out_edges) {
            if (!visited[edge->to]) {
                topologicalSortUtil(edge->to, visited, stack);
            }
        }

        stack.push(vertex->vid);
    }
};

#endif
