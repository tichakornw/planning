#include <cassert>
#include <iostream>

#include "RulebookCost.h"

void testRulebookCost(const Rulebook &rulebook) {
    std::cout << "Testing RulebookCost ..." << std::endl;
    RulebookCost::setRulebook(rulebook);
    RulebookCost cost1;
    RulebookCost cost2;
    cost1.setRuleCost(0, 2);
    cost1.setRuleCost(1, 1);
    cost2.setRuleCost(1, 2);
    cost2.setRuleCost(0, 1);
    bool res1 = cost1 <= cost2;
    bool res2 = cost2 <= cost1;
    bool res3 = cost1 == cost2;
    bool res4 = cost1 < cost2;
    bool res5 = cost1 > cost2;
    assert(res1);
    assert(!res2);
    assert(!res3);
    assert(res4);
    assert(!res5);

    RulebookCost cost3 = cost1 + cost2;
    assert(cost3[0]->getValue() == 3);
    assert(cost3[1]->getValue() == 2);

    std::cout << "Done" << std::endl;
}

void testRulebookCost2() {
    std::cout << "Testing RulebookCost2 ..." << std::endl;

    Rulebook rulebook;
    rulebook.addRule(RuleSum("r0"));
    rulebook.addRule(RuleSum("r1"));
    rulebook.build();

    RulebookCost::setRulebook(rulebook);
    // Add edges with costs
    RulebookCost cost44;
    cost44.setRuleCost(0, 4);
    cost44.setRuleCost(1, 4);

    RulebookCost cost1111;
    cost1111.setRuleCost(0, 11);
    cost1111.setRuleCost(1, 11);

    assert(cost44 <= cost1111);
    assert(cost44 < cost1111);
    std::cout << "Done" << std::endl;
}
