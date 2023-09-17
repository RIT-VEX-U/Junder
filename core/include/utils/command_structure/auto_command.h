/**
 * File: auto_command.h
 * Desc:
 *    Interface for module-specifc commands
 */

#pragma once

#include "vex.h"
#include <functional>
#include <vector>
#include <queue>

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

/// @brief Function is a quick and dirty Condition to wrap some expression that should be evaluated at runtime
class Function : public Condition
{
public:
  Function(std::function<bool()> cond) : cond(cond) {}
  bool test() override;

private:
  std::function<bool()> cond;
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

/// @brief FirstFinish runs multiple commands in parallel and exits when the first subcommand does
/// When one command finishes, on_timeout will be called on the rest in this command group and it continue
class FirstFinish : public AutoCommand
{
public:
  FirstFinish(std::vector<AutoCommand *> cmds);
  FirstFinish(std::initializer_list<AutoCommand *> cmds);
  bool run() override;
  void on_timeout() override;

private:
  std::vector<AutoCommand *> cmds;
  int finished_idx = -1;
  vex::timer tmr;
};

/// @brief  Parallel runs multiple commands in parallel and waits for all to finish before continuing.
/// if none finish before this command's timeout, it will call on_timeout on all children continue
class Parallel : public AutoCommand
{
public:
  Parallel(std::vector<AutoCommand *> cmds);
  Parallel(std::initializer_list<AutoCommand *> cmds);
  bool run() override;
  void on_timeout() override;

private:
  std::vector<AutoCommand *> cmds;
  std::vector<bool> finished;
  vex::timer tmr;
};

/// @brief Branch chooses from multiple options at runtime. the function decider returns an index into the choices vector
/// If you wish to make no choice and skip this section, return NO_CHOICE;
/// any choice that is out of bounds set to NO_CHOICE
class Branch : public AutoCommand
{
public:
  Branch(Condition *cond, AutoCommand *false_choice, AutoCommand *true_choice);
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
