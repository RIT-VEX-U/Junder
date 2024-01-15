#pragma once
#include <functional>
#include <memory>
#include <queue>
#include <string>
#include <type_traits>
#include <vector>

#include "core/utils/command_structure/condition.h"

// A memory managing wrapper for any autocommand. See below for more information
class AutoCommand;
/**
 * AutoCommandInterface is the interface of all commands for the auto system.
 * This class is virtual (member functions ending in = 0) so it can not be
 * instantiated. If you want to talk about a list of commands to run, use
 * AutoCommand
 */
class AutoCommandInterface {
  public:
    /**
     * The way to execute the command. returns true when this command is
     * finished
     */
    virtual bool run() = 0;
    /**
     * What to do when this command is interrupted
     * ie. stop motors, reset external state
     */
    virtual void on_timeout(){};
    /**
     * duplicates the command in its original state.
     * that is duplicate the command in the state it was before the first run()
     * was called.
     */
    virtual AutoCommand duplicate() const = 0;

    virtual ~AutoCommandInterface() {}
};

/// @brief TimeSinceStartExceeds tests based on time since the command
/// controller was constructed. Returns true if elapsed time > time_s
Condition TimeSinceStartExceeds(double seconds);

/// @brief Pauses until the condition is true. Basically delay but with a
/// condition instead of a time
AutoCommand PauseUntil(Condition cond);

/**
 * AutoCommand is a memory managed way to talk about autocommands.
 * Treat it just like you would an integer and it will take care of all the base
 * class nonsense See CommandController or InOrder for how this looks in
 * practice
 */
class AutoCommand {
  public:
    friend class CommandController;
    static constexpr double default_timeout = 10.0;
    static constexpr double DONT_TIMEOUT = -1.0;

    // Used to determine if the constructor should accept a certain command
    template <typename T>
    using IsAutoCommand = std::is_convertible<T *, AutoCommandInterface *>;

    /**
     * Implicit InOrder constructor. Helpful for grouping commands together
     * If you initilalize a command with a {} enclosed list of other
     * AutoCommands, this will be called
     */
    AutoCommand(std::initializer_list<AutoCommand> cmds);

    /**
     * Constructor from arbitrary command
     * Specialize this if you want to do something more fancy.
     * We specialize it for stuff like InOrder and Repeat where we don't want
     * the default timeout.
     *
     * @tparam CommandT a class that inherits from AutoCommandInterface
     * @param cmd the value of an AutoCommandInterface object
     */
    template <class CommandT> AutoCommand(CommandT cmd) {
        static_assert(
            !std::is_pointer<CommandT>::value,
            "Command should not be a pointer. We used to initilize command "
            "lists with pointers but you don't need to do that "
            "anymore");
        static_assert(IsAutoCommand<CommandT>::value,
                      "Command going into AutoCommand must "
                      "implement AutoCommandInterface");
        cmd_ptr = new CommandT(cmd);
    }

    // Special member functions. See .cpp for a longwinded explanation for why
    // these are needed
    AutoCommand(const AutoCommand &other);
    AutoCommand(AutoCommand &&other);
    AutoCommand operator=(const AutoCommand &other);
    AutoCommand operator=(AutoCommand &&other);

    ~AutoCommand();

    bool run();

    AutoCommand with_timeout(double seconds);
    AutoCommand until(Condition cond);
    bool does_timeout() const;
    void on_timeout();

  private:
    AutoCommandInterface *cmd_ptr = nullptr;
    Condition true_to_end = AlwaysFalseCondition();
    double timeout_seconds = default_timeout;
};

class InOrder : public AutoCommandInterface {
  public:
    InOrder();
    InOrder(std::initializer_list<AutoCommand> cmds);
    static InOrder FromVector(const std::vector<AutoCommand> &cmds);

    bool run() override;
    AutoCommand duplicate() const override;
    AutoCommand with_timeout(double seconds);
    AutoCommand until(Condition cond);
    InOrder repeat_times(size_t number_times);

  private:
    std::vector<AutoCommand> cmds;
};

class Repeat : public AutoCommandInterface {
  public:
    Repeat();
    Repeat(std::initializer_list<AutoCommand> cmds);
    static Repeat FromVector(const std::vector<AutoCommand> &cmds);
    bool run() override;
    AutoCommand duplicate() const override;
    AutoCommand with_timeout(double seconds);
    AutoCommand until(Condition cond);

  private:
    std::vector<AutoCommand> cmds;
    InOrder working_cmds;
};

class Branch : public AutoCommandInterface {
  public:
    Branch(Condition decider, AutoCommand iftrue, AutoCommand iffalse);
    bool run() override;
    AutoCommand duplicate() const override;

  private:
    bool is_decided;
    bool choice;
    Condition decider;
    AutoCommand iftrue;
    AutoCommand iffalse;
};

class Message : public AutoCommandInterface {
  public:
    Message(const std::string &msg);
    bool run() override;
    AutoCommand duplicate() const override;

  private:
    const std::string &msg;
};

class Parallel : public AutoCommandInterface {

  public:
    Parallel(std::initializer_list<AutoCommand> cmds);
    bool run() override;
    AutoCommand duplicate() const override;

  private:
    std::vector<AutoCommand> cmds;
};

class FunctionCommand : public AutoCommandInterface {
  public:
    FunctionCommand(std::function<bool()> f);
    bool run() override { return f(); }

    AutoCommand duplicate() const override;

  private:
    std::function<bool()> f;
};

// Specializations for commands for which the default makes no sens
template <> AutoCommand::AutoCommand(InOrder io);

template <> AutoCommand::AutoCommand(Repeat r);