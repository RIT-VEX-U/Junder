#include "core/utils/command_structure/condition.h"
#include "vex.h"

Condition ConditionBase::And(Condition c) {
    return fc([=]() { return this->test() && c->test(); });
}
Condition ConditionBase::Or(Condition c) {
    return fc([=]() { return this->test() || c->test(); });
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
Condition TimeSinceStartExceeds(double seconds) {
    // tmr is started at path creation time. which we say equals the start of
    // the path (true most of the time)
    return fc(
        [seconds, tmr = vex::timer()]() { return tmr.value() > seconds; });
}