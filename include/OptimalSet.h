#ifndef _OPTIMALSET_H
#define _OPTIMALSET_H

#include <iostream>
#include <unordered_set>

// Define the hash function for ElementWithCost
template <typename ElementType, typename CostType> struct ElementWithCost {
    ElementType element;
    CostType cost;
    size_t eid;

    bool operator==(const ElementWithCost &other) const {
        return eid == other.eid;
    }

    friend std::ostream &operator<<(std::ostream &os,
                                    const ElementWithCost &e) {
        os << "(Element: " << e.element << ", Cost: " << e.cost
           << ", ID: " << e.eid << ")";
        return os;
    }
};

template <typename ElementType, typename CostType> class OptimalSet {
  public:
    CostType getCost(size_t eid) { return optimal_elements[eid].cost; }

    ElementWithCost<ElementType, CostType> getElement(size_t eid) {
        return optimal_elements[eid];
    }

    bool isIn(size_t eid) {
        auto it = optimal_elements.find(eid);
        return it != optimal_elements.end();
    }

    std::unordered_set<size_t> insert(const ElementType &element,
                                      const CostType &cost, size_t eid) {
        std::unordered_set<size_t> removed_elements;
        for (auto it = optimal_elements.begin();
             it != optimal_elements.end();) {
            // Remove elements whose cost is greater than or equal to the given
            // cost
            if (cost < it->second.cost) {
                removed_elements.insert(it->first);
                it = optimal_elements.erase(
                    it); // Remove and move to next element
            }
            // If the given cost is larger than an existing element, we can
            // return empty set.
            else if (it->second.cost < cost) {
                return removed_elements;
            } else {
                ++it; // Move to next element
            }
        }

        optimal_elements[eid] =
            ElementWithCost<ElementType, CostType>{element, cost, eid};

        /* Previous implementation when optimal_elements is a vector
        optimal_elements.erase(
            std::remove_if(
                optimal_elements.begin(), optimal_elements.end(),
                [&](const auto &e) {
                    if (cost < e.cost) {
                        removed_elements.push_back(e);
                        return true; // Remove e from optimal_elements
                    }
                    return false; // Keep e in optimal_elements
                }),
            optimal_elements.end());

        // Check if the new element's cost is less than or equal to all
        // remaining elements' costs
        bool is_greater =
            std::any_of(optimal_elements.begin(), optimal_elements.end(),
                        [&](const auto &e) {
                            return e.cost < cost;
                        });

        if (!is_greater) {
            optimal_elements[eid] = ElementWithCost<ElementType,
        CostType>{element, cost, eid};
        }
        */

        return removed_elements;
    }

    friend std::ostream &operator<<(std::ostream &os, const OptimalSet &s) {
        auto it = s.optimal_elements.begin();
        auto end = s.optimal_elements.end();
        os << "[";
        while (it != end) {
            os << "{\"element\": " << it->second.element
               << ", \"cost\": " << it->second.cost
               << ", \"id\": " << it->second.eid << "}";

            ++it;
            if (it != end) {
                os << ", ";
            }
        }
        os << "]";
        return os;
    }

  private:
    std::unordered_map<size_t, ElementWithCost<ElementType, CostType>>
        optimal_elements;
};

#endif
