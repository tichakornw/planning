#ifndef _RULEBOOK_H
#define _RULEBOOK_H

#include <iostream>
#include <iterator>
#include <unordered_set>
#include <vector>
#include <cassert>

#include "Graph.h"
#include "Rule.h"

struct PairHash {
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1, T2> &p) const {
        return std::hash<T1>()(p.first) ^ (std::hash<T2>()(p.second) << 1);
    }
};

class Rulebook {
  public:
    class Iterator {
      public:
        using iterator_category = std::forward_iterator_tag;
        using value_type = std::unordered_set<size_t>;
        using difference_type = std::ptrdiff_t;
        using pointer = const std::unordered_set<size_t> *;
        using reference = const std::unordered_set<size_t> &; 

        Iterator(const std::vector<size_t> &sorted_indices,
                 const std::vector<std::unordered_set<size_t>> &sets,
                 std::size_t index)
            : sorted_indices(sorted_indices), sets(sets), index(index) {}

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

        bool operator==(const Iterator &other) const { return index == other.index; }
        bool operator!=(const Iterator &other) const { return !(*this == other); }

      private:
        const std::vector<size_t> &sorted_indices;
        const std::vector<std::unordered_set<size_t>> &sets;
        std::size_t index;
    };

  public:
    Rulebook();

    Iterator begin() const { return Iterator(sorted_rule_set, quotient_rule_set, 0); }
    Iterator end() const { return Iterator(sorted_rule_set, quotient_rule_set, sorted_rule_set.size()); }

    size_t addRule(const Rule &rule);
    void clear();
    const Rule &getRule(size_t id) const { return *rules[id]; }
    void setEquivalentClasses(const std::vector<std::unordered_set<size_t>> &eq_classes);
    void addGTRelation(size_t from_rule, size_t to_rule);
    size_t getNumRules() const { return rules.size(); }
    
    bool build();
    std::unordered_set<size_t> getSuccessors(size_t rule) const;
    bool isTotallyOrdered() const;
    bool isAllSingletons() const;
    virtual void display() const;

  private:
    int getQuotientIndex(size_t rule_index) const;
    bool isQuotientRuleSetComplete() const;
    std::vector<std::unordered_set<size_t>> completeQuotientRuleSet(
        const std::vector<std::unordered_set<size_t>> &eq_classes);

    std::vector<std::shared_ptr<Rule>> rules;
    std::vector<std::unordered_set<size_t>> quotient_rule_set;
    std::unordered_set<std::pair<size_t, size_t>, PairHash> rule_edges;

    Graph rule_graph;
    std::vector<size_t> sorted_rule_set;
    std::unordered_map<size_t, std::unordered_set<size_t>> rule_successors;
    bool is_built;
};

#endif