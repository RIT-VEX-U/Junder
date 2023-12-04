#include "core/utils/command_structure/condition.h"

Condition ConditionBase::And(Condition c) {
    return fc([&]() { return this->test() && c->test(); });
}
Condition ConditionBase::Or(Condition c) {
    return fc([&]() { return this->test() || c->test(); });
}

Condition fc(std::function<bool()> f){
    std::unique_ptr<ConditionBase> uptr = std::unique_ptr<FunctionCondition>(new FunctionCondition(f));
    return uptr;
}