#include <cassert>
#include <iostream>

#include "Rule.h"
#include "Rulebook.h"

void testRules() {
    RuleSum rsum("sumRule");
    auto a_ptr = rsum.makeCost(3.0);
    auto b_ptr = rsum.makeCost(4.0);

    const RuleCost &a = *a_ptr;
    const RuleCost &b = *b_ptr;

    const auto c = a + b;
    std::cout << "c = " << *c << std::endl;
    assert(c->getValue() == 7.0);

    RuleMax rmax("maxRule");
    auto x_ptr = rmax.makeCost(2.5);
    auto y_ptr = rmax.makeCost(9.1);
    const RuleCost &x = *x_ptr;
    const RuleCost &y = *y_ptr;

    const auto z = x + y;
    std::cout << "z = " << *z << std::endl;
    assert(z->getValue() == 9.1);

    auto w_ptr = rmax.makeCost(1.2);
    const RuleCost &w = *w_ptr;
    const RuleCost &v = *z;
    const auto u = v + w;
    assert(u->getValue() == 9.1);
}

Rulebook testRulebook(bool verbose = false) {
    Rulebook rulebook;

    RuleSum r0("r0");
    RuleMax r1("r1");
    RuleSum r2("r2");

    rulebook.addRule(r0);
    rulebook.addRule(r1);
    rulebook.addRule(r2);
    size_t r3id = rulebook.addRule(RuleMax("r3"));

    assert(r3id > 2);
    assert(rulebook.addRule(r2));
    assert(rulebook.addRule(r1));

    rulebook.setEquivalentClasses({{1, 2, r3id}, {0, 4}});

    bool exceptionThrown = false;
    try {
        rulebook.addGTRelation(2, 6);
    } catch (const std::exception &e) {
        exceptionThrown = true;
    }
    assert(exceptionThrown &&
           "Expected exception not thrown by addGTRelation(2, 7)");
    try {
        rulebook.addGTRelation(2, r3id);
    } catch (const std::exception &e) {
        exceptionThrown = true;
    }
    assert(exceptionThrown &&
           "Expected exception not thrown by addGTRelation(2, r3id)");

    rulebook.addGTRelation(2, 5);
    rulebook.addGTRelation(1, 4);

    rulebook.build();

    rulebook.display();

    std::vector<std::unordered_set<size_t>> classes;
    if (verbose)
        std::cout << "Order: ";
    for (auto it = rulebook.begin(); it != rulebook.end(); ++it) {
        std::unordered_set<size_t> s(it->begin(), it->end());
        classes.push_back(s);
        if (verbose) {
            std::cout << "{ ";
            for (auto elem : *it) {
                std::cout << elem << " ";
                std::cout << "} ";
            }
        }
    }
    if (verbose)
        std::cout << std::endl;
    // There should be exactly 3 sets
    assert(classes.size() == 3);

    // Check that the first set is exactly {1, 2, r3id}, the second set is
    // either {0, 5} or {6}
    std::unordered_set<size_t> set04 = {0, 4};
    std::unordered_set<size_t> set5 = {5};
    assert(classes[0] == std::unordered_set<size_t>({1, 2, r3id}));
    assert((classes[1] == set04 and classes[2] == set5) or
           (classes[1] == set5 and classes[2] == set04));

    // Successors
    std::unordered_set<size_t> set045 = {0, 4, 5};
    std::unordered_set<size_t> set_empty;
    if (verbose)
        std::cout << "Successors:" << std::endl;
    for (size_t i = 0; i < 8; ++i) {
        const auto successors = rulebook.getSuccessors(i);
        if (i != 1 && i != 2 && i != r3id)
            assert(successors == set_empty);
        else
            assert(successors == set045);
        if (verbose) {
            std::cout << "  " << i << ": ";
            for (auto j : successors) {
                std::cout << j << " ";
            }
            std::cout << std::endl;
        }
    }
    return rulebook;
}
