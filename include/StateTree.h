#pragma once

#include "StateGraph.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

template <typename State, typename CostType, typename StateTransition>
class StateTree : public StateGraph<State, CostType, StateTransition> {
  public:
    using Base = StateGraph<State, CostType, StateTransition>;
    using typename Base::SEdge;
    using typename Base::SEdgePtr;
    using typename Base::SVertex;
    using typename Base::SVertexPtr;

  private:
    // child -> parent mapping
    std::unordered_map<size_t, size_t> parent_of_;
    // cost-to-come (g value)
    std::unordered_map<size_t, CostType> cost_to_come_;

  public:
    StateTree() = default;
    ~StateTree() override = default;

    // -----------------------------------------------------------
    // Add vertex with default cost-to-come = infinity
    // -----------------------------------------------------------
    size_t addStateVertex(const State &state, bool is_root = false) {
        size_t vid = Base::addStateVertex(state);
        cost_to_come_[vid] =
            is_root ? CostType(0)
                    : CostType(std::numeric_limits<CostType>::infinity());
        return vid;
    }

    // -----------------------------------------------------------
    // Add edge ensuring tree property and updating costs
    // -----------------------------------------------------------
    size_t addStateTransition(size_t from, size_t to, const CostType &cost,
                              const StateTransition &transition,
                              bool allow_duplicate = true,
                              bool check_cycle = false) {
        if (parent_of_.count(to))
            throw std::runtime_error("StateTree: vertex already has a parent");
        if (from == to)
            throw std::runtime_error(
                "StateTree: cannot connect vertex to itself");
        if (check_cycle && createsCycle(from, to))
            throw std::runtime_error(
                "StateTree: adding edge would create a cycle");

        size_t eid = Base::addStateTransition(from, to, cost, transition,
                                              allow_duplicate);
        parent_of_[to] = from;

        // Update cost-to-come
        CostType parent_cost = cost_to_come_[from];
        cost_to_come_[to] = parent_cost + cost;

        return eid;
    }

    // -----------------------------------------------------------
    // Rewire (RRT* style)
    // -----------------------------------------------------------
    void rewire(size_t new_parent, size_t child, const CostType &new_cost,
                const StateTransition &new_transition,
                bool check_cycle = false) {
        auto it_old = parent_of_.find(child);
        if (it_old == parent_of_.end())
            throw std::runtime_error(
                "StateTree: cannot rewire root or orphan node");

        size_t old_parent = it_old->second;
        if (new_parent == child)
            throw std::runtime_error(
                "StateTree: cannot rewire a node to itself");
        if (check_cycle && createsCycle(new_parent, child))
            throw std::runtime_error(
                "StateTree: rewiring would create a cycle");

        // --- Find and remove the existing edge (old_parent → child) ---
        auto old_parent_v = this->getStateVertex(old_parent);
        auto child_v = this->getStateVertex(child);
        if (!old_parent_v || !child_v)
            throw std::runtime_error("StateTree: invalid vertex in rewire");
        std::shared_ptr<Edge> edge_to_remove = nullptr;
        for (const auto &edge : child_v->in_edges) {
            if (edge->from == old_parent_v) {
                edge_to_remove = edge;
                break;
            }
        }

        if (!edge_to_remove)
            throw std::runtime_error(
                "StateTree: cannot find old edge to remove");

        // --- Remove edge safely using base class helper ---
        Base::removeEdge(edge_to_remove);

        // Add new edge
        Base::addStateTransition(new_parent, child, new_cost, new_transition,
                                 true);

        // Update parent
        parent_of_[child] = new_parent;

        // Update cost-to-come recursively
        CostType new_g = cost_to_come_[new_parent] + new_cost;
        propagateCostUpdate(child_v, new_g);
    }

    // -----------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------
    SVertexPtr getParent(size_t vid) const {
        auto it = parent_of_.find(vid);
        if (it == parent_of_.end())
            return nullptr;
        return this->getStateVertex(it->second);
    }

    CostType getCostToCome(size_t vid) const {
        auto it = cost_to_come_.find(vid);
        return (it != cost_to_come_.end())
                   ? it->second
                   : std::numeric_limits<CostType>::infinity();
    }

    bool isRoot(size_t vid) const { return !parent_of_.count(vid); }

    // -----------------------------------------------------------
    // Debug
    // -----------------------------------------------------------
    void printTree() const {
        std::cout << "StateTree:\n";
        for (auto &[vid, vertex] : this->vertices) {
            auto parent = getParent(vid);
            std::cout << "  v" << vid;
            if (parent)
                std::cout << " <- v" << parent->vid;
            else
                std::cout << " (root)";
            std::cout << ", g=" << getCostToCome(vid) << "\n";
        }
    }

  private:
    bool createsCycle(size_t from, size_t to) const {
        size_t current = from;
        while (parent_of_.count(current)) {
            current = parent_of_.at(current);
            if (current == to)
                return true;
        }
        return false;
    }

    // Recursively propagate cost update to all descendants
    void propagateCostUpdate(const SVertexPtr &vertex, CostType new_cost) {
        cost_to_come_[vertex->vid] = new_cost;
        for (auto &edge : vertex->out_edges) {
            auto child_edge = std::dynamic_pointer_cast<SEdge>(edge);
            if (!child_edge)
                continue;

            auto child = std::dynamic_pointer_cast<SVertex>(child_edge->to);
            if (!child)
                continue;

            propagateCostUpdate(child, new_cost + child_edge->cost);
        }
    }
};
