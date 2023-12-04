#pragma once
#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "core/utils/command_structure/condition.h"

class AutoCommand;
class AutoCommandBase {
   public:
    // The way to execute the command. returns true when this command is
    // finished
    virtual bool run() { return true; };
    // What to do when this command is interrupted
    // clean up after yourself.
    virtual void on_timeout(){};
    /// duplicates the command in its original state.
    /// that is duplicate the command in the state it was before the first run()
    /// was called.
    virtual AutoCommand duplicate() const = 0;
    virtual ~AutoCommandBase() {}
};

// Handles memory management
// A custom unique ptr basically
class AutoCommand {
   public:
    const double default_timeout = 10.0;
    const double DONT_TIMEOUT = -1.0;

    // Implicit InOrder constructor. Helpful for grouping commands together
    AutoCommand(std::initializer_list<AutoCommand> cmds);

    // constructor from arbitrary command
    template <typename CommandT>
    AutoCommand(CommandT cmd) {
        static_assert(
            !std::is_pointer<CommandT>::value,
            "Command should not be a pointer. We used to initilize command "
            "lists with pointers but you don't need to do that "
            "anymore");
        static_assert(std::is_convertible<CommandT *, AutoCommandBase *>::value,
                      "Command going into AutoCommand must "
                      "implement AutoCommandBase");
        cmd_ptr = new CommandT(cmd);
    }

    // Special Constructors. See .cpp for why these are needed
    AutoCommand(const AutoCommand &other);
    AutoCommand(AutoCommand &&other);
    AutoCommand operator=(const AutoCommand &other);
    AutoCommand operator=(AutoCommand &&other);

    ~AutoCommand();

    bool run();

    AutoCommand withTimeout(double seconds);
    AutoCommand until(Condition &&cond);

   private:
    AutoCommandBase *cmd_ptr = nullptr;
    Condition true_to_end = AlwaysFalseCondition();
    double timeout_seconds = default_timeout;
};

class FunctionCommand : public AutoCommandBase {
   public:
    FunctionCommand(std::function<bool()> f) : f(f) {}
    bool run() override { return f(); }

    AutoCommand duplicate() const override {
        return AutoCommand(FunctionCommand(f));
    }

   private:
    std::function<bool()> f;
};

class InOrder : public AutoCommandBase {
   public:
    InOrder();
    InOrder(std::initializer_list<AutoCommand> cmds);
    bool run() override;
    AutoCommand duplicate() const override;
    AutoCommand withTimeout(double seconds);
    AutoCommand until(Condition &&cond);

   private:
    std::vector<AutoCommand> cmds;
};

class Repeat : public AutoCommandBase {
   public:
    Repeat();
    Repeat(std::initializer_list<AutoCommand> cmds);
    bool run() override;
    AutoCommand duplicate() const override;
    AutoCommand withTimeout(double seconds);
    AutoCommand until(Condition &&cond);

   private:
    std::vector<AutoCommand> cmds;
};

/// @brief TimeSinceStartExceeds tests based on time since the command
/// controller was constructed. Returns true if elapsed time > time_s
// Condition TimeSinceStartExceeds(double seconds) {
//     return fc([=, tmr = vex::timer()]() { return tmr.value() > seconds; });
// }

/// @brief Pauses until the condition is true. Basically delay but with a
/// condition instead of a time
Condition PauseUntilCondition(Condition &&cond);
