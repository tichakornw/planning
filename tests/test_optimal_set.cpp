#include <cassert>
#include <iostream>
#include <vector>

#include "OptimalSet.h"
#include "Plan.h"

void testOptimalSet() {
    OptimalSet<Plan, double> optimal_set;
    std::vector<int> element1 = {1, 2, 3};
    std::vector<int> element2 = {2, 2, 4};
    std::vector<int> element3 = {2, 3, 4};
    std::vector<int> element4 = {2, 3, 4};

    Plan p1(element1);
    Plan p2(element2);
    Plan p3(element3);
    Plan p4(element4);

    optimal_set.insert(p1, 10.0, 1);
    optimal_set.insert(p1, 8.0, 2);
    optimal_set.insert(p2, 5.0, 3);
    optimal_set.insert(p3, 15.0, 4);
    optimal_set.insert(p3, 5.0, 5);

    assert(optimal_set.getNumElements() == 2);
    assert(optimal_set.isIn(3));
    assert(optimal_set.isIn(5));

    const auto el3 = optimal_set.getElement(3);
    const auto el5 = optimal_set.getElement(5);
    assert(el3.element == p2);
    assert(el5.element == p3);
    assert(el3.cost == 5.0);
    assert(el5.cost == 5.0);
    assert(el3.eid == 3);
    assert(el5.eid == 5);

    // Print the elements in the optimal set
    std::cout << optimal_set << std::endl;
}
