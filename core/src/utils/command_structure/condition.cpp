#include "core/utils/command_structure/condition.h"

Condition ConditionBase::And(Condition c) {
    return fc([&]() { return this->test() && c->test(); });
}
Condition ConditionBase::Or(Condition c) {
    return fc([&]() { return this->test() || c->test(); });
}

Condition fc(std::function<bool()> f) {
    return std::shared_ptr<ConditionBase>(new FunctionCondition(f));
}

Condition AlwaysFalseCondition() {
    return fc([]() { return false; });
}
Condition AlwaysTrueCondition() {
    return fc([]() { return true; });
}