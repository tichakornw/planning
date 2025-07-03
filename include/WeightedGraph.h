#ifndef _WEIGHTEDGRAPH_H
#define _WEIGHTEDGRAPH_H

#include "Graph.h"
#include "OptimalSet.h"
#include <memory>
#include <unordered_set>
#include <vector>
#include <limits>
#include <functional>

template <typename CostType> class WeightedEdge : public Edge {
  public:
    CostType cost;

    WeightedEdge(const std::shared_ptr<Vertex> &v1,
                 const std::shared_ptr<Vertex> &v2, const CostType &cost,
                 size_t eid)
        : Edge(v1, v2, eid), cost(cost) {}

    void setCost(CostType cost) { cost = cost; }
};

template <typename CostType>
std::ostream &
operator<<(std::ostream &os,
           const std::vector<std::shared_ptr<WeightedEdge<CostType>>> &edges) {
    os << "[";
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto &edge = edges[i];
        if (edge) {
            os << *edge;
        } else {
            os << "null";
        }
        if (i + 1 != edges.size()) {
            os << ", ";
        }
    }
    os << "]";
    return os;
}

template <typename CostType> class WeightedGraph : public Graph {
  public:
    using WEdge = WeightedEdge<CostType>;
    using VertexPtr = std::shared_ptr<Vertex>;
    using WEdgePtr = std::shared_ptr<WEdge>;

    // Add an edge with weight to the graph
    void addEdge(size_t from, size_t to, const CostType &cost, size_t eid) {
        auto v1 = getVertex(from);
        auto v2 = getVertex(to);

        if (v1 && v2) {
            auto edge = std::make_shared<WEdge>(v1, v2, cost, eid);
            addEdgeToGraph(v1, v2, edge);
        }
    }

    // Display the graph with weights
    void display() const override {
        for (const auto &pair : vertices) {
            std::cout << "  \"" << pair.first << "\": [";
            for (size_t i = 0; i < pair.second->out_edges.size(); ++i) {
                auto edge = pair.second->out_edges[i];
                auto weighted_edge = std::dynamic_pointer_cast<WEdge>(edge);
                if (weighted_edge) {
                    std::cout << "[\"" << weighted_edge->to->vid << "\", "
                              << weighted_edge->cost << "]";
                } else {
                    std::cout << "[\"" << edge->to->vid << "\"]";
                }
                if (i != pair.second->out_edges.size() - 1) {
                    std::cout << ", ";
                }
            }
            std::cout << "]" << std::endl;
        }
    }

    // To call the following function, do
    // graph.reduceToOptimalSubgraph<double>(0, 5,
    // [](const WEdgePtr& wedge) {
    //    auto wedge0 = std::dynamic_pointer_cast<WEdge>(wedge);
    //    return wedge0->cost * 1.2; // e.g., inflated cost
    //});

    template<
        typename SubCostType = CostType,
        typename GetSubCost = std::function<SubCostType(const WEdgePtr&)>
    >
    void
    reduceToOptimalSubgraph(size_t from, size_t to,
                            bool use_max,
                            GetSubCost getSubCost = [](const WEdgePtr& wedge) {
                                auto wedge0 = std::dynamic_pointer_cast<WEdge>(wedge);
                                return wedge0->cost;
                            }) {
        const auto vI = getVertex(from);
        const auto vG = getVertex(to);
        if (!vI || !vG)
            throw std::invalid_argument("Invalid initial or goal vertex");

        const auto cost_to_come = this->getCostToComeFrom<SubCostType>(vI, use_max, false, getSubCost);
        std::cout << "cost to go" << std::endl;
        const auto cost_to_go = this->getCostToComeFrom<SubCostType>(vG, use_max, true, getSubCost);

        const auto optimal_cost = cost_to_come.at(to);

        std::cout << "Optimal cost: " << optimal_cost << std::endl;

        // Determine whether a given edge is part of optimal subgraph
        auto is_edge_optimal = [&](const WEdgePtr& wedge) {
            size_t from_id = wedge->from->vid;
            size_t to_id = wedge->to->vid;
            SubCostType total;
            if (use_max)
                total = std::max({cost_to_come.at(from_id), wedge->cost, cost_to_go.at(to_id)});
            else
                total = cost_to_come.at(from_id) + wedge->cost + cost_to_go.at(to_id);

            return total < optimal_cost + 1e-6;
        };

        // Remove edges
        auto it = std::remove_if(edges.begin(), edges.end(), [&](const std::shared_ptr<Edge>& edge) {
            const auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
            if (!wedge) return false;

            if (is_edge_optimal(wedge)) return false;

            // Remove from vertices as a side effect
            RemoveEdgeFromVertices(edge);  // removes from from->out_edges and to->in_edges
            return true;       // mark for removal from top-level `edges`
        });

        edges.erase(it, edges.end());

        /*
        auto it = std::remove_if(edges.begin(), edges.end(), [&](const std::shared_ptr<Edge>& edge) {
            const auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
            if (!wedge)
                return false;
            if (is_edge_optimal(wedge))
                return false;

            // Remove from vertex->out_edges and in_edges
            auto& out_vec = wedge->from->out_edges;
            out_vec.erase(std::remove(out_vec.begin(), out_vec.end(), wedge), out_vec.end());

            auto& in_vec = wedge->to->in_edges;
            in_vec.erase(std::remove(in_vec.begin(), in_vec.end(), wedge), in_vec.end());

            return true;  // mark for removal from top-level `edges`
        });

        edges.erase(it, edges.end());
        */
    }

    OptimalSet<std::vector<WEdgePtr>, CostType>
    getOptimalPaths(size_t from, size_t to) const {
        const auto vI = getVertex(from);
        const auto vG = getVertex(to);
        if (!vI || !vG || edges.empty())
            return OptimalSet<std::vector<WEdgePtr>, CostType>();

        size_t n = 0;

        // Initialize optimal_paths
        const auto wedge0 = std::dynamic_pointer_cast<WEdge>(edges[0]);
        const CostType zero_cost = wedge0->cost.getZero();

        std::unordered_map<VertexPtr,
                           OptimalSet<std::vector<WEdgePtr>, CostType>>
            optimal_paths;
        for (const auto &pair : vertices) {
            optimal_paths[pair.second] =
                OptimalSet<std::vector<WEdgePtr>, CostType>();
            if (pair.second == vI)
                optimal_paths[pair.second].insert(std::vector<WEdgePtr>(),
                                                  zero_cost, n);
        }

        // Initialize maps
        std::unordered_map<size_t, VertexPtr> vertices_t2g;

        Graph ST;
        ST.addVertex(n);
        vertices_t2g[n] = vI;
        std::queue<size_t> Q;
        Q.push(n);

        while (!Q.empty()) {
            const size_t vtid = Q.front();
            Q.pop();
            const auto current_vertex = vertices_t2g[vtid];
            const auto current_path_with_cost =
                optimal_paths[current_vertex].getElement(vtid);

            for (const auto &edge : current_vertex->out_edges) {
                const auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
                const auto next_vertex = edge->to;
                std::vector<WEdgePtr> next_path =
                    current_path_with_cost.element;
                next_path.push_back(wedge);
                // std::cout << "vtid: " << vtid << " cost " <<
                // current_path_with_cost.cost;
                const auto next_cost =
                    current_path_with_cost.cost + wedge->cost;
                // std::cout << " next cost: " << next_cost << std::endl;
                const std::unordered_set<size_t> removed_vtids =
                    optimal_paths[next_vertex].insert(next_path, next_cost,
                                                      n + 1);
                removeElementsFromQueue(Q, removed_vtids);
                if (optimal_paths[next_vertex].isIn(n + 1)) {
                    ++n;
                    ST.addVertex(n);
                    ST.addEdge(vtid, n, edge->eid);
                    vertices_t2g[n] = next_vertex;
                    Q.push(n);
                }
            }
        }

        return optimal_paths[vG];
    }

  private:
    void
    removeElementsFromQueue(std::queue<size_t> &Q,
                            const std::unordered_set<size_t> &to_remove) const {
        std::queue<size_t> newQueue;

        while (!Q.empty()) {
            size_t element = Q.front();
            Q.pop();

            // Check if the element is in the to_remove set
            if (to_remove.find(element) == to_remove.end()) {
                newQueue.push(element);
            }
        }

        // Swap the new queue with the original queue
        Q.swap(newQueue);
    }

    template<typename SubCostType>
    std::unordered_map<size_t, SubCostType>
    getCostToComeFrom(const VertexPtr vI,
                      bool use_max,
                      bool reverse_graph,
                      std::function<SubCostType(const WEdgePtr&)> getSubCost) const {
        std::unordered_map<size_t, SubCostType> cost_to_come;

        // Initialize all costs to "infinity"
        SubCostType INF = std::numeric_limits<SubCostType>::max();
        for (const auto &pair : vertices) {
            cost_to_come[pair.first] = INF;
        }

        // Dijkstra queue: (cost, vertex ID)
        using QElem = std::pair<SubCostType, VertexPtr>;
        std::priority_queue<QElem, std::vector<QElem>, std::greater<>> pq;

        cost_to_come[vI->vid] = SubCostType{};  // assume SubCostType{} = zero
        pq.emplace(SubCostType{}, vI);

        while (!pq.empty()) {
            auto [current_cost, current_vertex] = pq.top();
            std::cout << "  current_cost " << current_cost << " vertex " << current_vertex->vid << std::endl;
            pq.pop();

            if (current_cost > cost_to_come[current_vertex->vid])
                continue;

            const auto& vertex_edges = reverse_graph ? current_vertex->in_edges : current_vertex->out_edges;

            for (const auto &edge : vertex_edges) {
                const auto wedge = std::dynamic_pointer_cast<WEdge>(edge);
                const auto next_vertex = reverse_graph ? edge->from : edge->to;
                std::cout << "    next vertex: " << next_vertex->vid << std::endl;
                SubCostType edge_cost = getSubCost(wedge);
                SubCostType new_cost = use_max ? std::max(current_cost, edge_cost) : current_cost + edge_cost;

                if (new_cost < cost_to_come[next_vertex->vid]) {
                    cost_to_come[next_vertex->vid] = new_cost;
                    pq.emplace(new_cost, next_vertex);
                }
            }
        }

        return cost_to_come;
    }

};

#endif
