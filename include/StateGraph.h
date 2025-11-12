#pragma once

#include "Vertex.h"
#include "WeightedGraph.h"
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

template <typename State> class StateVertex : public Vertex {
  public:
    State state;

  public:
    StateVertex(const State &s, size_t vid) : Vertex(vid), state(s) {}

    friend std::ostream &operator<<(std::ostream &os, const StateVertex &v) {
        os << "Vertex[" << v.vid << "]";
        if constexpr (requires(std::ostream & os2, const State &st) {
                          os2 << st;
                      })
            os << " " << v.state;
        return os;
    }
};

template <typename State, typename CostType, typename StateTransition>
class StateTransitionEdge : public WeightedEdge<CostType> {
  public:
    StateTransition transition;

    StateTransitionEdge(const std::shared_ptr<Vertex> &v1,
                        const std::shared_ptr<Vertex> &v2, const CostType &cost,
                        const StateTransition &transition, size_t eid)
        : WeightedEdge<CostType>(v1, v2, cost, eid), transition(transition) {}

    std::shared_ptr<StateVertex<State>> fromState() const {
        return std::static_pointer_cast<StateVertex<State>>(this->from);
    }

    std::shared_ptr<StateVertex<State>> toState() const {
        return std::static_pointer_cast<StateVertex<State>>(this->to);
    }

    friend std::ostream &operator<<(std::ostream &os,
                                    const StateTransitionEdge &e) {
        os << "Edge[" << e.eid << "] " << *e.from << " -> " << *e.to
           << " cost=" << e.cost;
        return os;
    }
};

template <typename State, typename CostType, typename StateTransition>
void toJsonStreamEdges(
    std::ostream &os,
    const std::vector<
        std::shared_ptr<StateTransitionEdge<State, CostType, StateTransition>>>
        &edges) {
    os << std::fixed << std::setprecision(6);
    os << "[\n";
    for (size_t i = 0; i < edges.size(); ++i) {
        const auto &e = edges[i];
        if (!e)
            continue;

        const auto &t = e->transition;
        const auto &s1 = t.getStartState();
        const auto &s2 = t.getEndState();

        os << "  {\"id\": " << e->eid << ", \"x1\": " << s1.x
           << ", \"y1\": " << s1.y << ", \"x2\": " << s2.x
           << ", \"y2\": " << s2.y << ", \"cost\": " << e->cost << "}";
        if (i + 1 < edges.size())
            os << ",";
        os << "\n";
    }
    os << "]";
}

template <typename State, typename CostType, typename StateTransition>
class StateGraph : public WeightedGraph<CostType> {
  public:
    using Base = WeightedGraph<CostType>;
    using SVertex = StateVertex<State>;
    using SVertexPtr = std::shared_ptr<SVertex>;

    using SEdge = StateTransitionEdge<State, CostType, StateTransition>;
    using SEdgePtr = std::shared_ptr<SEdge>;

  public:
    StateGraph() = default;
    ~StateGraph() override = default;

    /**
     * @brief Add vertex from a State
     *
     */
    size_t addStateVertex(const State &state) {
        size_t vid = this->vertices.size();
        auto vertex = std::make_shared<SVertex>(state, vid);
        this->vertices[vid] = vertex;
        return vid;
    }

    /**
     * @brief Add edge as a state transition
     *
     */
    size_t addStateTransition(size_t from, size_t to, const CostType &cost,
                              const StateTransition &transition,
                              bool allow_duplicate = true) {
        auto v1 = this->getVertex(from);
        auto v2 = this->getVertex(to);

        if (!v1)
            std::cout << "Can't find from vertex " << from << std::endl;
        if (!v2)
            std::cout << "Can't find to vertex " << to << std::endl;

        assert(v1 && v2);

        size_t eid = this->edges.size();
        auto edge = std::make_shared<SEdge>(v1, v2, cost, transition, eid);
        this->addEdgeToGraph(v1, v2, edge, allow_duplicate);
        return eid;
    }

    /**
     * @brief Get nearest vertex (for planning)
     *
     */
    SVertexPtr nearestVertex(const State &query) const {
        double best_dist = std::numeric_limits<double>::infinity();
        SVertexPtr best_vertex = nullptr;

        for (const auto &[vid, vertex] : this->vertices) {
            auto sv = std::dynamic_pointer_cast<SVertex>(vertex);
            if (!sv)
                continue;

            double d = query.distance(sv->state);
            if (d < best_dist) {
                best_dist = d;
                best_vertex = sv;
            }
        }
        return best_vertex;
    }

    /**
     * @brief Type-safe getters
     *
     */
    SVertexPtr getStateVertex(size_t vid) const {
        auto v = this->getVertex(vid);
        return std::dynamic_pointer_cast<SVertex>(v);
    }

    /**
     * @brief Export graph structure as JSON
     *
     */
    void toJsonStream(std::ostream &os) const {
        os << std::fixed << std::setprecision(6);
        os << "{\n";

        // --- Vertices ---
        size_t i = 0;
        os << "  \"vertices\": [\n";
        for (const auto &[vid, vertex_base] : this->vertices) {
            auto v = std::dynamic_pointer_cast<SVertex>(vertex_base);
            if (!v) {
                std::cout << "Can't cast " << vid << std::endl;
                std::cout << "Type: " << typeid(*vertex_base).name()
                          << std::endl;
                continue;
            }

            // Assumes State has members x and y
            os << "    {\"id\": " << v->vid << ", \"x\": " << v->state.x
               << ", \"y\": " << v->state.y << "}";
            if (++i < this->vertices.size())
                os << ",";
            os << "\n";
        }
        os << "  ],\n";

        // --- Edges ---
        os << "  \"edges\": [\n";
        for (size_t i = 0; i < this->edges.size(); ++i) {
            auto e = std::dynamic_pointer_cast<SEdge>(this->edges[i]);
            if (!e)
                continue;
            const auto &t = e->transition;

            os << "    {\"id\": " << e->eid << ", \"from\": " << e->from->vid
               << ", \"to\": " << e->to->vid
               << ", \"x1\": " << t.getStartState().x
               << ", \"y1\": " << t.getStartState().y
               << ", \"x2\": " << t.getEndState().x
               << ", \"y2\": " << t.getEndState().y << ", \"cost\": " << e->cost
               << "}";
            if (i + 1 < this->edges.size())
                os << ",";
            os << "\n";
        }
        os << "  ]\n";

        os << "}";
    }
};
