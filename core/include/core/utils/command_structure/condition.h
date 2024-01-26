#pragma once
#include <functional>
#include <memory>

/**
 * A Condition is a function that returns true or false
 * is_even is a predicate that would return true if a number is even
 * For our purposes, a Condition is a choice to be made at runtime
 * drive_sys.reached_point(10, 30) is a predicate
 * time.has_elapsed(10, vex::seconds) is a predicate
 * extend this class for different choices you wish to make
 */

class ConditionBase;

using Condition = std::shared_ptr<ConditionBase>;

/// Base interface for a condition. Most important is test()
class ConditionBase {
  public:
    virtual bool test() = 0;

    Condition Or(Condition c);
    Condition And(Condition c);
    virtual ~ConditionBase() {}
};

class FunctionCondition : public ConditionBase {
  public:
    FunctionCondition(std::function<bool()> f) : f(f) {}
    bool test() override { return true; }

  private:
    std::function<bool()> f;
};

// shorthand for function Condition
Condition fc(std::function<bool()> f);

Condition AlwaysFalseCondition();
Condition AlwaysTrueCondition();

/// @brief TimeSinceStartExceeds tests based on time since the command
/// controller was constructed. Returns true if elapsed time > time_s
Condition TimeSinceStartExceeds(double seconds);

Condition TimeSinceStartExceeds(double seconds) {
    // tmr is started at path creation time. which we say equals the start of
    // the path (true most of the time)
    return fc(
        [seconds, tmr = vex::timer()]() { return tmr.value() > seconds; });
}