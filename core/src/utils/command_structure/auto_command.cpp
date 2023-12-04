#include "core/utils/command_structure/auto_command.h"

#include <stdio.h>

#include "vex.h"

// Auto command implementation =================================
bool AutoCommand::run() {
    if (cmd_ptr == nullptr) {
        printf("Called run on null command\n");
        return true;
    }
    return cmd_ptr->run();
}

AutoCommand AutoCommand::with_timeout(double seconds) {
    timeout_seconds = seconds;
    return *this;
}
void AutoCommand::on_timeout() {
    if (cmd_ptr == nullptr) {
        printf("trying to call on_timeout() on null autocommand\n");
        return;
    }
    return cmd_ptr->on_timeout();
}

bool AutoCommand::does_timeout() { return timeout_seconds != DONT_TIMEOUT; }

AutoCommand::AutoCommand(std::initializer_list<AutoCommand> cmds) {
    this->cmd_ptr = new InOrder(cmds);
    this->timeout_seconds = DONT_TIMEOUT;
}

AutoCommand AutoCommand::until(Condition cond) {
    true_to_end = true_to_end->Or(std::move(cond));
    return *this;
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

InOrder::InOrder() : cmds{} {}
InOrder::InOrder(std::initializer_list<AutoCommand> cmds) : cmds{cmds} {}

bool InOrder::run() {
    printf("Inorder::run() not implemented\n");
    return true;
}

AutoCommand InOrder::duplicate() const {
    InOrder other;  // all the correct copying happens magically  :)
    other.cmds = cmds;
    auto ac = AutoCommand(other);
    return other.with_timeout(AutoCommand::DONT_TIMEOUT);
    ;
}
AutoCommand InOrder::with_timeout(double seconds) {
    return AutoCommand(*this).with_timeout(seconds);
}

AutoCommand InOrder::until(Condition cond) {
    return AutoCommand(*this)
        .until(std::forward<Condition>(cond))
        .with_timeout(AutoCommand::DONT_TIMEOUT);
}

InOrder InOrder::repeat_times(size_t times) {
    cmds.reserve(times * cmds.size());
    size_t og_size = cmds.size();
    for (size_t i = 0; i < times - 1; i++) {
        for (size_t j = 0; j < og_size; j++) {
            cmds.push_back(cmds[j]);
        }
    }
    return *this;
}
template <>
AutoCommand::AutoCommand(InOrder io) {
    AutoCommand ac = io.duplicate();
    this->cmd_ptr = ac.cmd_ptr;
    ac.cmd_ptr = nullptr;
    this->timeout_seconds = DONT_TIMEOUT;
}

Repeat::Repeat() : cmds{} {}
Repeat::Repeat(std::initializer_list<AutoCommand> cmds)
    : cmds{cmds}, working_cmds{cmds} {}

Repeat Repeat::FromVector(const std::vector<AutoCommand> &cmds) {
    Repeat r;
    r.cmds = cmds;
    for (const AutoCommand &cmd : cmds) {
        r.working_cmds.push(cmd);
    }
    return r;
}

bool Repeat::run() {
    printf("unimplemented Repeat::run()\n");
    return true;
}

AutoCommand Repeat::duplicate() const { return Repeat::FromVector(cmds); }
AutoCommand Repeat::with_timeout(double seconds) {
    return AutoCommand(*this).with_timeout(seconds);
}

AutoCommand Repeat::until(Condition cond) {
    return AutoCommand(*this).until(std::forward<Condition>(cond));
}

template <>
AutoCommand::AutoCommand(Repeat r) {
    AutoCommand ac = r.duplicate();
    this->cmd_ptr = ac.cmd_ptr;
    ac.cmd_ptr = nullptr;
    this->timeout_seconds = DONT_TIMEOUT;
}

Condition TimeSinceStartExceeds(double seconds) {
    return fc(
        [seconds, tmr = vex::timer()]() { return tmr.value() > seconds; });
}

AutoCommand FunctionCommand::duplicate() const {
    return AutoCommand(FunctionCommand(f));
}

AutoCommand PauseUntil(Condition c) {
    return FunctionCommand([&]() { return c->test(); });
}
