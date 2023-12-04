#pragma once
/**
 * File: command_controller.h
 * Desc:
 *    A CommandController manages the AutoCommands that make
 *    up an autonomous route. The AutoCommands are kept in
 *    a queue and get executed and removed from the queue
 *    in FIFO order.
 */
#include <vector>

#include "core/utils/command_structure/auto_command.h"

class CommandController {
   public:
    CommandController(std::initializer_list<AutoCommand> cmds);
    void run();
    void set_cancel_func(std::function<bool()> true_to_end);

   private:
    std::vector<AutoCommand> cmds;
    std::function<bool()> controller_should_end = [](){return false;}; 
};
