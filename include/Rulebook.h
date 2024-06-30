#ifndef _RULEBOOK_H
#define _RULEBOOK_H

#include <iostream>
#include <iterator>
#include <unordered_set>
#include <vector>

#include "Graph.h"

class Rulebook {
  public:
    class Iterator {
      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = std::unordered_set<size_t>;
        using difference_type = std::ptrdiff_t;
        using pointer = const std::unordered_set<size_t> *;
        using reference =
            const std::unordered_set<size_t> &; // Use const reference

        Iterator(const std::vector<size_t> &sorted_indices,
                 const std::vector<std::unordered_set<size_t>> &sets,
                 std::size_t index)
            : sorted_indices(sorted_indices), sets(sets), index(index) {}

        // Iterator operations
        reference operator*() const { return sets[sorted_indices[index]]; }

        pointer operator->() const { return &sets[sorted_indices[index]]; }

        Iterator &operator++() {
            ++index;
            return *this;
        }

        Iterator operator++(int) {
            Iterator temp = *this;
            ++(*this);
            return temp;
        }

        bool operator==(const Iterator &other) const {
            return index == other.index;
        }

        bool operator!=(const Iterator &other) const {
            return !(*this == other);
        }

      private:
        const std::vector<size_t> &sorted_indices;
        const std::vector<std::unordered_set<size_t>> &sets;
        std::size_t index;
    };

  public:
    Rulebook() : num_rules(0), is_built(false) {}

    Iterator begin() const {
        return Iterator(sorted_rule_set, quotient_rule_set, 0);
    }

    Iterator end() const {
        return Iterator(sorted_rule_set, quotient_rule_set,
                        sorted_rule_set.size());
    }

    bool addEquivalentRules(const std::unordered_set<size_t> &equiv_rules) {
        for (auto &rule_set : quotient_rule_set) {
            for (auto rule : rule_set) {
                if (equiv_rules.find(rule) != equiv_rules.end()) {
                    return false;
                }
            }
        }
        for (auto &rule : equiv_rules) {
            if (rule >= num_rules) {
                num_rules = rule + 1;
            }
        }
        quotient_rule_set.push_back(equiv_rules);
        is_built = false;
        return true;
    }

    bool addGTRelation(size_t from_rule, size_t to_rule) {
        int from_rule_index = getQuotientIndex(from_rule);
        int to_rule_index = getQuotientIndex(to_rule);
        if (from_rule_index < 0 || to_rule_index < 0)
            return false;
        std::pair<size_t, size_t> edge =
            std::make_pair((size_t)from_rule_index, (size_t)to_rule_index);
        rule_edges.push_back(edge);
        is_built = false;
        return true;
    }

    size_t getNumRules() const { return num_rules; }

    bool build() {
        addMissingRules();
        for (size_t vid = 0; vid < quotient_rule_set.size(); ++vid) {
            rule_graph.addVertex(vid);
        }
        size_t eid = 0;
        for (const auto &edge : rule_edges) {
            rule_graph.addEdge(edge.first, edge.second, eid);
            ++eid;
        }
        sorted_rule_set = rule_graph.topologicalSort();

        for (size_t vid = 0; vid < quotient_rule_set.size(); ++vid) {
            const auto vertex = rule_graph.getVertex(vid);

            // Check that rule_graph is a DAG
            for (auto edge : vertex->out_edges) {
                auto it_vid =
                    find(sorted_rule_set.begin(), sorted_rule_set.end(), vid);
                auto it_suc = find(sorted_rule_set.begin(),
                                   sorted_rule_set.end(), edge->to->vid);
                // If parent vertex does not appear first, then there is a cycle
                if (it_suc - it_vid < 0) {
                    return false;
                }
            }

            // Construct rule_successors
            const auto successors = rule_graph.findSuccessors(vid);
            for (const auto &rule : quotient_rule_set[vid]) {
                for (const auto &suc_vid : successors) {
                    rule_successors[rule].insert(
                        quotient_rule_set[suc_vid].begin(),
                        quotient_rule_set[suc_vid].end());
                }
            }
        }
        is_built = true;

        return true;
    }

    std::unordered_set<size_t> getSuccessors(size_t rule) const {
        auto it = rule_successors.find(rule);
        if (it != rule_successors.end()) {
            return it->second;
        } else {
            return {}; // Return an empty unordered_map
        }
    }

    // Display the rulebook
    virtual void display() const {
        std::cout << "Rules: ";
        for (const auto &rule_set : quotient_rule_set) {
            std::cout << "{ ";
            for (const auto rule : rule_set) {
                std::cout << rule << " ";
            }
            std::cout << "} ";
        }
        std::cout << std::endl;
        std::cout << "Edges: ";
        for (const auto &edge : rule_edges) {
            std::cout << "(" << edge.first << ", " << edge.second << ") ";
        }
        std::cout << std::endl;
        if (is_built) {
            std::cout << "Graph:" << std::endl;
            rule_graph.display();
        }
    }

  private:
    int getQuotientIndex(size_t rule_index) const {
        for (size_t i = 0; i < quotient_rule_set.size(); ++i) {
            if (quotient_rule_set[i].find(rule_index) !=
                quotient_rule_set[i].end())
                return i;
        }
        return -1;
    }

    void addMissingRules() {
        for (size_t rule_index = 0; rule_index < num_rules; ++rule_index) {
            if (getQuotientIndex(rule_index) < 0)
                quotient_rule_set.push_back({rule_index});
        }
    }

    std::vector<std::unordered_set<size_t>> quotient_rule_set;
    std::vector<std::pair<size_t, size_t>> rule_edges;

    size_t num_rules;
    Graph rule_graph;
    std::vector<size_t> sorted_rule_set;
    std::unordered_map<size_t, std::unordered_set<size_t>> rule_successors;
    bool is_built;
};

#endif
