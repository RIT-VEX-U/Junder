#include "core/utils/command_structure/auto_command.h"

#include <stdio.h>

// Auto command implementation =================================
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

AutoCommand::AutoCommand(std::initializer_list<AutoCommand> cmds) {
    this->cmd_ptr = new InOrder(cmds);
    this->timeout_seconds = DONT_TIMEOUT;
}

/***
 * Special Member functions
 * TODO EXPLAIN THESE
 */
AutoCommand::AutoCommand(const AutoCommand &other) {
    AutoCommand dupe = other.cmd_ptr->duplicate();
    this->cmd_ptr = dupe.cmd_ptr;
    this->timeout_seconds = dupe.timeout_seconds;
    dupe.cmd_ptr = nullptr;  // so that our reference isnt deleted when dupe
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

Repeat::Repeat() : cmds{} {}
Repeat::Repeat(std::initializer_list<AutoCommand> cmds) : cmds{cmds} {}

bool Repeat::run() {
    printf("unimplemented Repeat::run()\n");
    return true;
}

AutoCommand Repeat::duplicate() const {
    Repeat other;
    other.cmds = cmds;  // all the correct copying happens magically  :)
    return other;
}

AutoCommand Repeat::withTimeout(double seconds) {
    return AutoCommand(*this).withTimeout(seconds);
}

Condition PauseUntilCondition(Condition &&cond) {
    return fc([&]() { return cond->test(); });
}
