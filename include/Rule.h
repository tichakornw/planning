#include <iostream>

class Rule {
  public:
    Rule(const std::string &name, double value = 0)
        : name(name), value(value) {}

    // Overloading the stream insertion operator for easy printing
    friend std::ostream &operator<<(std::ostream &os, const Rule &rule) {
        os << rule.name << ": " << rule.value;
        return os;
    }

    // Overloading comparison operators
    bool operator<(const Rule &other) const { return value < other.value; }

    bool operator>(const Rule &other) const { return value > other.value; }

    bool operator<=(const Rule &other) const { return value <= other.value; }

    bool operator>=(const Rule &other) const { return value >= other.value; }

    bool operator==(const Rule &other) const { return value == other.value; }

  private:
    std::string name;
    double value;
};
