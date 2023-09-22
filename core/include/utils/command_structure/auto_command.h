/**
 * File: auto_command.h
 * Desc:
 *    Interface for module-specifc commands
 */


// DoWhile(action, condition)
// WaitUntilCondition()
#pragma once

#include "vex.h"
#include <functional>
#include <vector>
#include <queue>
#include <atomic>

class AutoCommand
{
public:
  static constexpr double default_timeout = 10.0;
  /**
   * Executes the command
   * Overridden by child classes
   * @returns true when the command is finished, false otherwise
   */
  virtual bool run() { return true; }
  /**
   * What to do if we timeout instead of finishing. timeout is specified by the timeout seconds in the constructor
   */
  virtual void on_timeout() {}
  AutoCommand *withTimeout(double t_seconds)
  {
    if (this->timeout_seconds < 0){
      // should never be timed out
      return this;
    }
    this->timeout_seconds = t_seconds;
    return this;
  }
  /**
   * How long to run until we cancel this command.
   * If the command is cancelled, on_timeout() is called to allow any cleanup from the function.
   * If the timeout_seconds <= 0, no timeout will be applied and this command will run forever
   * A timeout can come in handy for some commands that can not reach the end due to some physical limitation such as
   * - a drive command hitting a wall and not being able to reach its target
   * - a command that waits until something is up to speed that never gets up to speed because of battery voltage
   * - something else...
   */
  double timeout_seconds = default_timeout;
};

/**
 * A Condition is a function that returns true or false
 * is_even is a predicate that would return true if a number is even
 * For our purposes, a Condition is a choice to be made at runtime
 * drive_sys.reached_point(10, 30) is a predicate
 * time.has_elapsed(10, vex::seconds) is a predicate
 * extend this class for different choices you wish to make

 */
class Condition
{
public:
  virtual bool test() = 0;
};

/// @brief FunctionCondition is a quick and dirty Condition to wrap some expression that should be evaluated at runtime
class FunctionCondition : public Condition
{
public:
  FunctionCondition(
      std::function<bool()> cond, std::function<void(void)> timeout = []() {}) : cond(cond), timeout(timeout)
  {
  }
  bool test() override;

private:
  std::function<bool()> cond;
  std::function<void(void)> timeout;
};

/// @brief IfTimePassed tests based on time since the command controller was constructed. Returns true if elapsed time > time_s
class IfTimePassed : public Condition
{
public:
  IfTimePassed(double time_s);
  bool test() override;

private:
  double time_s;
  vex::timer tmr;
};

/// @brief InOrder runs its commands sequentially then continues.
/// How to handle timeout in this case. Automatically set it to sum of commands timouts?
class InOrder : public AutoCommand
{
public:
  InOrder(std::queue<AutoCommand *> cmds);
  InOrder(std::initializer_list<AutoCommand *> cmds);
  bool run() override;
  void on_timeout() override;

private:
  AutoCommand *current_command = nullptr;
  std::queue<AutoCommand *> cmds;
  vex::timer tmr;
};

/// @brief  Parallel runs multiple commands in parallel and waits for all to finish before continuing.
/// if none finish before this command's timeout, it will call on_timeout on all children continue
class Parallel : public AutoCommand
{
public:
  Parallel(std::initializer_list<AutoCommand *> cmds);
  bool run() override;
  void on_timeout() override;

private:
  std::vector<AutoCommand *> cmds;
  std::vector<vex::task *> runners;
};

/// @brief Branch chooses from multiple options at runtime. the function decider returns an index into the choices vector
/// If you wish to make no choice and skip this section, return NO_CHOICE;
/// any choice that is out of bounds set to NO_CHOICE
class Branch : public AutoCommand
{
public:
  Branch(Condition *cond, AutoCommand *false_choice, AutoCommand *true_choice);
  ~Branch();
  bool run() override;
  void on_timeout() override;

private:
  AutoCommand *false_choice;
  AutoCommand *true_choice;
  Condition *cond;
  bool choice = false;
  bool chosen = false;
  vex::timer tmr;
};

/// @brief Async runs a command asynchronously
/// will simply let it go and never look back
/// THIS HAS A VERY NICHE USE CASE. THINK ABOUT IF YOU REALLY NEED IT
class Async : public AutoCommand
{
public:
  Async(AutoCommand *cmd) : cmd(cmd) {}
  bool run() override;

private:
  AutoCommand *cmd;
};

/// @brief Awaitable provides a way to start a command and set it loose.
/// Use with caution, often Parallel is what you're looking for
/// If you do use this, calling await (to wait for command to run its course), or
/// stop() to explicitly give up on the command is recommended to avoid
/// IF YOU ARE USING THIS, CONSIDER USING PARALLEL?
/// Think about it at least
class Awaitable
{
public:
  /// @brief Creates a new awaitable. 
  /// use this.start() in the command controller to set it loose 
  /// use this.await() in the command controller to wait upon it to finish
  /// THIS AN ONLY BE USED ONCE
  /// @param cmd 
  Awaitable(AutoCommand *cmd);
  /// @brief begins the awaiting command
  /// @return the command that does this
  AutoCommand *start();
  /// @brief waits for the command to finish
  /// or if specified, waits for that many seconds before moving on
  /// @return the command that does this
  AutoCommand *await(double timeout = -1.0);

private:
  AutoCommand *cmd = nullptr;
  vex::task *runner = nullptr;
  std::atomic<bool> is_done;

  // dont look at these, they are for Awaitable to use
  void beginner();
  bool checker();
  void ender();

};