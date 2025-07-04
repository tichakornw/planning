#ifndef _RULE_COST_H
#define _RULE_COST_H
#include <iostream>

class RuleCost {
 public:
    RuleCost(double v = 0)
        : value(v) {}

    virtual ~RuleCost() = default;

    // This is to allow allow polymorphic copying to
    // allow RulebookCost to store its derived classes in a single container.
    virtual std::shared_ptr<RuleCost> clone() const = 0;

    virtual std::shared_ptr<RuleCost> operator+(const RuleCost& other) const = 0;

    // Overloading the stream insertion operator for easy printing
    friend std::ostream &operator<<(std::ostream &os, const RuleCost &rc) {
        os << rc.value;
        return os;
    }

    // Overloading comparison operators
    bool operator<(const RuleCost &other) const { return value < other.value; }

    bool operator>(const RuleCost &other) const { return value > other.value; }

    bool operator<=(const RuleCost &other) const { return value <= other.value; }

    bool operator>=(const RuleCost &other) const { return value >= other.value; }

    bool operator==(const RuleCost &other) const { return value == other.value; }

    void setValue(double v) {
        value = v;
    }

    double getValue() const { return value; }

  protected:
    double value;
};

class RuleCostMax : public RuleCost {
  public:
    RuleCostMax(double v = 0) : RuleCost(v) {}

    std::shared_ptr<RuleCost> clone() const override {
        return std::make_shared<RuleCostMax>(*this);
    }

    std::shared_ptr<RuleCost> operator+(const RuleCost& other) const override {
        return std::make_shared<RuleCostMax>(std::max(value, other.getValue()));
    }

    /*
    std::shared_ptr<RuleCostMax> operator+(const RuleCostMax& other) const {
        return std::make_shared<RuleCostMax>(std::max(value, other.value));
    }
    */

    RuleCostMax operator+(const RuleCostMax& other) const {
        return RuleCostMax(std::max(value, other.value));
    }
};

class RuleCostSum : public RuleCost {
  public:
    RuleCostSum(double v = 0) : RuleCost(v) {}

    std::shared_ptr<RuleCost> clone() const override {
        return std::make_shared<RuleCostSum>(*this);
    }

    std::shared_ptr<RuleCost> operator+(const RuleCost& other) const override {
        return std::make_shared<RuleCostSum>(value + other.getValue());
    }

    /*
    std::shared_ptr<RuleCostSum> operator+(const RuleCostSum& other) const {
        return std::make_shared<RuleCostSum>(value + other.value);
    }
    */

    RuleCostSum operator+(const RuleCostSum& other) const {
        return RuleCostSum(value + other.value);
    }
};


#endif
