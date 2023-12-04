#include "../core/include/utils/command_structure/auto_command.h"
#include <stdio.h>

bool AutoCommand::run() {
    if (cmd_ptr == nullptr) {
        printf("Called run on null command\n");
        return true;
    }
    return cmd_ptr->run();
}

AutoCommand AutoCommand::withTimeout(double seconds) {
    timeout_seconds = seconds;
    return *this;
}

AutoCommand::AutoCommand(const AutoCommand &other) {
    AutoCommand dupe = other.cmd_ptr->duplicate();
    this->cmd_ptr = dupe.cmd_ptr;
    this->timeout_seconds = dupe.timeout_seconds;
    dupe.cmd_ptr = nullptr; // so that our reference isnt deleted when dupe
                            // goes out of scope
}

AutoCommand::AutoCommand(AutoCommand &&other)
    : cmd_ptr(other.cmd_ptr), timeout_seconds(other.timeout_seconds) {
    other.cmd_ptr = nullptr;
}

AutoCommand AutoCommand::operator=(const AutoCommand &other) {
    return AutoCommand(other);
}

AutoCommand AutoCommand::operator=(AutoCommand &&other) {
    return AutoCommand(other);
}

AutoCommand::~AutoCommand() {
    if (cmd_ptr == nullptr) {
        return;
    }
    delete cmd_ptr;
}

Condition PauseUntilCondition(Condition &&cond) {
    return fc([&]() { return cond->test(); });
}