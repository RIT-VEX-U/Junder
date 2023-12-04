#pragma once
#include <functional>
#include <memory>

// Backported from C++14
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args &&...args) {
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

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

template <typename Cond>
Condition ConditionFrom(Cond c) {
    static_assert(std::is_convertible<Cond *, ConditionBase *>::value,
                  "Cond must implement ConditionBase");
    std::unique_ptr<ConditionBase> uptr = c;
    return uptr;
}

class ConditionBase {
   public:
    Condition Or(Condition c);
    Condition And(Condition c);
    virtual bool test() = 0;
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
