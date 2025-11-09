#include "LinearTransition.h"
#include "Point2D.h"
#include "StateGraph.h"
#include <cassert>
#include <iostream>
#include <memory>
#include <sstream>

void testStateVertexEdge() {
    using CostType = double;
    using State = Point2D;
    using SV = StateVertex<State>;
    using STE = StateTransitionEdge<State, CostType, LinearTransition<State>>;

    // Create vertices
    auto p1 = Point2D(0.0, 0.0);
    auto p2 = Point2D(1.0, 1.0);

    auto v1 = std::make_shared<SV>(p1, 0);
    auto v2 = std::make_shared<SV>(p2, 1);

    assert(v1->vid == 0);
    assert(v2->vid == 1);
    assert(v1->state.x == 0.0 && v1->state.y == 0.0);
    assert(v2->state.x == 1.0 && v2->state.y == 1.0);

    // Check operator<< for StateVertex
    std::ostringstream vout;
    vout << *v1;
    std::string vstr = vout.str();
    std::cout << "Vertex output: " << vstr << "\n";
    assert(vstr.find("Vertex[0]") != std::string::npos);
    assert(vstr.find("[0,0]") != std::string::npos);

    // Create an edge
    auto e = std::make_shared<STE>(v1, v2, 5.5, LinearTransition(p1, p2), 42);

    assert(e->eid == 42);
    assert(e->cost == 5.5);
    assert(e->transition.getStartState() == p1);
    assert(e->transition.getEndState() == p2);
    assert(e->from == v1);
    assert(e->to == v2);

    // Verify fromState() and toState() casting
    auto fsv = e->fromState();
    auto tsv = e->toState();
    assert(fsv == v1);
    assert(tsv == v2);
    assert(fsv->state.x == 0.0);
    assert(tsv->state.x == 1.0);

    // Stream output for edge
    std::ostringstream eout;
    eout << *e;
    std::string estr = eout.str();
    std::cout << "Edge output: " << estr << "\n";
    assert(estr.find("Edge[42]") != std::string::npos);
    assert(estr.find("cost=5.5") != std::string::npos);
    assert(estr.find("0 -> 1") != std::string::npos);

    // Cleanup
    e->clear();
    v1->clear(); // clears out_edges
    v2->clear(); // clears in_edges
}

void testStateGraph() {
    using State = Point2D;
    using CostType = double;
    using Transition = LinearTransition<State>;

    // Create the graph
    StateGraph<State, CostType, Transition> graph;

    // Add vertices
    size_t v0 = graph.addStateVertex({0, 0});
    size_t v1 = graph.addStateVertex({1, 0});
    size_t v2 = graph.addStateVertex({1, 1});
    size_t v3 = graph.addStateVertex({0, 1});

    // Add edges with LinearTransition
    auto tr0 = Transition(graph.getStateVertex(v0)->state,
                          graph.getStateVertex(v1)->state);
    auto tr1 = Transition(graph.getStateVertex(v1)->state,
                          graph.getStateVertex(v2)->state);
    auto tr2 = Transition(graph.getStateVertex(v2)->state,
                          graph.getStateVertex(v3)->state);
    auto tr3 = Transition(graph.getStateVertex(v3)->state,
                          graph.getStateVertex(v0)->state);

    graph.addStateTransition(v0, v1, tr0.getLength(), tr0);
    graph.addStateTransition(v1, v2, tr1.getLength(), tr1);
    graph.addStateTransition(v2, v3, tr2.getLength(), tr2);
    graph.addStateTransition(v3, v0, tr3.getLength(), tr3);

    // Verify vertices and edges
    for (size_t vid = 0; vid < 4; ++vid) {
        auto sv = graph.getStateVertex(vid);
        assert(sv != nullptr);
        std::cout << "Vertex " << vid << ": " << sv->state << "\n";
    }

    std::cout << "Edges:\n";
    for (auto &edge : graph.getEdges()) {
        std::cout << "  " << *edge << "\n";
    }

    // Test nearestVertex
    Point2D query{0.9, 0.05};
    auto nearest = graph.nearestVertex(query);
    assert(nearest != nullptr);
    std::cout << "Nearest vertex to " << query << ": " << nearest->state
              << "\n";
}
