#include "core/utils/command_structure/command_controller.h"

#include <stdio.h>

#include "vex.h"

CommandController::CommandController(std::initializer_list<AutoCommand> cmds)
    : cmds{cmds} {}

void CommandController::run() {
    for (AutoCommand &cmd : cmds) {
        bool cmd_finished = false;
        bool force_finished = false;
        vex::timer cmd_timer;

        while (!cmd_finished) {
            cmd_finished = cmd.run();

            if (cmd.does_timeout() && cmd_timer.value() > cmd.timeout_seconds) {
                cmd_finished = true;
                force_finished = true;
            }
            if (cmd.true_to_end->test()) {
                cmd_finished = true;
                force_finished = true;
            }

            if (controller_should_end()) {
                cmd.on_timeout();
                return;
            }

            vexDelay(10);
        }
        if (force_finished) {
            cmd.on_timeout();
        }
    }
}
void CommandController::set_cancel_func(std::function<bool()> true_to_end) {
    controller_should_end = true_to_end;
}