#include "Rulebook.h"
#include <stdexcept>
#include <algorithm>

Rulebook::Rulebook() : is_built(false) {}

size_t Rulebook::addRule(const Rule &rule) {
    size_t rid = rules.size();
    rules.push_back(rule.clone());
    quotient_rule_set.emplace_back(std::unordered_set<size_t>{rid});
    return rid;
}

void Rulebook::clear() {
    rules.clear();
    quotient_rule_set.clear();
    rule_edges.clear();
    rule_graph.clear();
    sorted_rule_set.clear();
    rule_successors.clear();
    is_built = false;
}

void Rulebook::setEquivalentClasses(const std::vector<std::unordered_set<size_t>> &eq_classes) {
    if (!rule_edges.empty()) {
        throw std::logic_error("Cannot set equivalence classes after GT relations have been added.");
    }
    quotient_rule_set = completeQuotientRuleSet(eq_classes);
    is_built = false;
}

void Rulebook::addGTRelation(size_t from_rule, size_t to_rule) {
    int from_rule_index = getQuotientIndex(from_rule);
    if (from_rule_index < 0) {
        throw std::invalid_argument("from_rule " + std::to_string(from_rule) + " not found in rulebook.");
    }

    int to_rule_index = getQuotientIndex(to_rule);
    if (to_rule_index < 0) {
        throw std::invalid_argument("to_rule " + std::to_string(to_rule) + " not found in rulebook.");
    }

    if (from_rule_index == to_rule_index) {
        throw std::invalid_argument("rules are of the same rank ");
    }

    std::pair<size_t, size_t> backward_edge = {to_rule_index, from_rule_index};
    if (rule_edges.find(backward_edge) != rule_edges.end()) {
        throw std::invalid_argument("Rule " + std::to_string(to_rule) +
                                    " is of higher rank than " + std::to_string(from_rule));
    }

    std::pair<size_t, size_t> edge = std::make_pair((size_t)from_rule_index, (size_t)to_rule_index);
    rule_edges.insert(edge);
    is_built = false;
}

bool Rulebook::build() {
    if (is_built)
        std::cerr << "Warning: rulebook is already built!" << std::endl;

    if (!isQuotientRuleSetComplete()) {
        throw std::logic_error("Cannot build because the set of equivalent classes is not complete.");
    }

    rule_graph.clear();
    for (size_t vid = 0; vid < quotient_rule_set.size(); ++vid) {
        rule_graph.addVertex(vid);
    }
    for (const auto &edge : rule_edges) {
        rule_graph.addEdge(edge.first, edge.second);
    }
    sorted_rule_set = rule_graph.topologicalSort();

    for (size_t vid = 0; vid < quotient_rule_set.size(); ++vid) {
        const auto vertex = rule_graph.getVertex(vid);

        for (auto edge : vertex->out_edges) {
            auto it_vid = std::find(sorted_rule_set.begin(), sorted_rule_set.end(), vid);
            auto it_suc = std::find(sorted_rule_set.begin(), sorted_rule_set.end(), edge->to->vid);
            if (it_suc - it_vid < 0) {
                return false;
            }
        }

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

std::unordered_set<size_t> Rulebook::getSuccessors(size_t rule) const {
    assert(is_built);
    auto it = rule_successors.find(rule);
    if (it != rule_successors.end()) {
        return it->second;
    }
    return {}; 
}

bool Rulebook::isTotallyOrdered() const {
    assert(is_built);
    if (!isAllSingletons()) return false;
    return rule_graph.isTotallyOrdered();
}

bool Rulebook::isAllSingletons() const {
    for (const auto &eq_class : quotient_rule_set) {
        if (eq_class.size() != 1) return false;
    }
    return true;
}

void Rulebook::display() const {
    std::cout << "Rules: ";
    for (size_t id = 0; id < this->getNumRules(); ++id) {
        if (id > 0) std::cout << ", ";
        std::cout << "(" << id << ", " << *rules[id] << ")";
    }
    std::cout << std::endl << "Equivalent class: ";
    for (const auto &rule_set : quotient_rule_set) {
        std::cout << "{ ";
        for (const auto rule : rule_set) {
            std::cout << rule << " ";
        }
        std::cout << "} ";
    }
    std::cout << std::endl << "Edges: ";
    for (const auto &edge : rule_edges) {
        std::cout << "(" << edge.first << ", " << edge.second << ") ";
    }
    std::cout << std::endl;
    if (is_built) {
        std::cout << "Graph:" << std::endl;
        rule_graph.display();
    }
}

int Rulebook::getQuotientIndex(size_t rule_index) const {
    for (size_t i = 0; i < quotient_rule_set.size(); ++i) {
        if (quotient_rule_set[i].find(rule_index) != quotient_rule_set[i].end())
            return i;
    }
    return -1;
}

bool Rulebook::isQuotientRuleSetComplete() const {
    std::unordered_set<size_t> seen;
    for (const auto &eq_class : quotient_rule_set) {
        for (size_t id : eq_class) {
            if (id >= rules.size()) {
                std::cerr << "Invalid rule ID: " << id << " (out of bounds)" << std::endl;
                return false;
            }
            if (!seen.insert(id).second) {
                std::cerr << "Duplicate rule ID in quotient_rule_set: " << id << std::endl;
                return false;
            }
        }
    }
    if (seen.size() != rules.size()) {
        std::cerr << "Missing rules in quotient_rule_set." << std::endl;
        return false;
    }
    return true;
}

std::vector<std::unordered_set<size_t>> Rulebook::completeQuotientRuleSet(
    const std::vector<std::unordered_set<size_t>> &eq_classes) {
    std::unordered_set<size_t> seen_rids;
    for (const auto &eq_class : eq_classes) {
        for (size_t rid : eq_class) {
            if (rid >= this->getNumRules()) {
                throw std::invalid_argument("Rule ID " + std::to_string(rid) + " not found in rulebook.");
            }
            if (!seen_rids.insert(rid).second) {
                throw std::invalid_argument("Duplicate rule ID " + std::to_string(rid) + " in equivalence classes.");
            }
        }
    }
    std::vector<std::unordered_set<size_t>> completed_classes = eq_classes;
    for (size_t rid = 0; rid < this->getNumRules(); ++rid) {
        if (seen_rids.find(rid) == seen_rids.end()) {
            completed_classes.emplace_back(std::unordered_set<size_t>{rid});
        }
    }
    return completed_classes;
}