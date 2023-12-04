#pragma once
/**
 * File: command_controller.h
 * Desc:
 *    A CommandController manages the AutoCommands that make
 *    up an autonomous route. The AutoCommands are kept in
 *    a queue and get executed and removed from the queue
 *    in FIFO order.
 */
#include "../core/include/utils/command_structure/auto_command.h"
#include <vector>

class CommandController {
  public:
    CommandController(std::initializer_list<AutoCommand> cmds) {}
    void run();
    void add_cancel_func(std::function<bool()>) {}
};
