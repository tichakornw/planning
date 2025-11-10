#pragma once

#include "StateGraph.h"
#include <algorithm>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

/**
 * @brief A directed acyclic tree built on top of StateGraph, maintaining
 *        parent-child relationships and cost-to-come values.
 *
 * @tparam State           Type representing a configuration or position in the
 * state space
 * @tparam CostType        Numeric type for edge and cumulative path costs
 * @tparam StateTransition Type representing a valid motion between two states
 *
 * This class extends StateGraph by enforcing the *tree property*:
 *  - Each vertex (except the root) has exactly one parent.
 *  - Edges implicitly encode a parent → child hierarchy.
 *
 * Additionally, it maintains:
 *  - A mapping from each vertex to its parent (`parent_of_`).
 *  - The cost-to-come (`g-value`) from the root for each vertex.
 *
 * The tree supports RRT* operations such as rewiring and dynamic cost
 * propagation.
 */
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

    /**
     * @brief Add a new vertex to the tree.
     * @param state   The state to insert.
     * @param is_root Whether this vertex is the root (g=0).
     * @return The vertex ID of the newly added state.
     */
    size_t addStateVertex(const State &state, bool is_root = false) {
        size_t vid = Base::addStateVertex(state);
        cost_to_come_[vid] =
            is_root ? CostType(0)
                    : CostType(std::numeric_limits<CostType>::infinity());
        return vid;
    }

    /**
     * @brief Add a new directed edge from `from` -> `to` and update
     * cost-to-come.
     *
     * Ensures that each vertex has only one parent (tree structure).
     * Optionally checks for cycles if desired.
     *
     * @throws std::runtime_error if attempting to connect an existing child,
     *         create a self-loop, or introduce a cycle.
     */
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

    // -------------------------------------------------------------------------
    // Tree Structure Queries
    // -------------------------------------------------------------------------

    /**
     * @brief Retrieve the edge from a vertex’s parent to itself.
     * @return Shared pointer to the parent edge, or nullptr if none exists.
     */
    std::shared_ptr<StateTransitionEdge<State, CostType, StateTransition>>
    getParentEdge(size_t vid) const {
        auto v = this->getStateVertex(vid);
        if (!v || v->in_edges.empty())
            return nullptr;
        assert(v->in_edges.size() == 1);
        return std::dynamic_pointer_cast<
            StateTransitionEdge<State, CostType, StateTransition>>(
            v->in_edges.front());
    }

    /**
     * @brief Get the parent vertex of a given node.
     * @return Shared pointer to the parent vertex, or nullptr if root.
     */
    SVertexPtr getParent(size_t vid) const {
        auto it = parent_of_.find(vid);
        if (it == parent_of_.end())
            return nullptr;
        return this->getStateVertex(it->second);
    }

    /**
     * @brief Get the accumulated cost-to-come from the root.
     */
    CostType getCostToCome(size_t vid) const {
        auto it = cost_to_come_.find(vid);
        return (it != cost_to_come_.end())
                   ? it->second
                   : std::numeric_limits<CostType>::infinity();
    }

    /** @brief Return true if the vertex is the root (no parent). */
    bool isRoot(size_t vid) const { return !parent_of_.count(vid); }

    // -------------------------------------------------------------------------
    // Rewiring (RRT* operation)
    // -------------------------------------------------------------------------

    /**
     * @brief Change a node’s parent and update costs recursively.
     *
     * Used in RRT* when a shorter path to a node is discovered.
     * Automatically removes the old edge, inserts the new one, and
     * propagates cost-to-come updates to all descendants.
     */
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

    // -------------------------------------------------------------------------
    // Debugging / Visualization
    // -------------------------------------------------------------------------

    /** @brief Print the tree structure and cost-to-come of all vertices. */
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
    // -------------------------------------------------------------------------
    // Internal Utilities
    // -------------------------------------------------------------------------

    /**
     * @brief Detect if adding an edge from → to would create a cycle.
     *
     * Used defensively when building or rewiring the tree.
     */
    bool createsCycle(size_t from, size_t to) const {
        size_t current = from;
        while (parent_of_.count(current)) {
            current = parent_of_.at(current);
            if (current == to)
                return true;
        }
        return false;
    }

    /**
     * @brief Recursively propagate a cost update through all descendants.
     *
     * Called after rewiring to maintain consistent g-values across the tree.
     */
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
