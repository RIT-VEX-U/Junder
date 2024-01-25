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
double AutoCommand::timeout_time() { return timeout_seconds; }

bool AutoCommand::does_timeout() const {
    return timeout_seconds != DONT_TIMEOUT;
}

AutoCommand AutoCommand::until(Condition cond) {
    true_to_end = true_to_end->Or(std::move(cond));
    return *this;
}

/***
 * Special Member functions
 *
 * C++ uses RAII (resource aquisition is initialization) to manage memory, or
 * other resources that need to be 'opened' and 'closed' (databases, mutexes,
 * and more - oh my!)
 *
 * Resources:
 * - <a href = "https://en.cppreference.com/w/cpp/language/raii">RAII</a>
 * - <a href =
 * "https://en.cppreference.com/w/cpp/language/move_constructor">Move
 * Semantics</a>
 *
 * AutoCommand uses this concept to handle holding classes of the
 * AutoCommandInterfaceType. Since it is an interface/base class, we must use a
 * pointer to access its members. Though we could statically allocate them all
 * and point to global memory, this would have terrible user experience so we
 * allocate them on the heap using `new`. We used to do this by hand which had
 * many disadvantages
 * - one of every 5 words in your auto path was the word new
 * - everything was a pointer and you never knew if you were the only person
 * using the object at the end of that pointer
 * - it was really really easy to leak memory. If you tried to delete the
 * memory, if anyone else as looking at it, you would segfault
 *
 * Ugly, misperforming, and leaky code were really quite annoying so we
 * separated AutoCommand and AutoCommandInterface to memory manage for you
 * Using AutoCommand, new is no longer necessary (template constructor accepts
 * non pointer objects and manages the allocations automatically).
 * Memory management is also solved using a technique very similar to <a href=
 * "std::unique_ptr">std::unique_ptr</a>. One notable difference is that a
 * std::unique_ptr has no `copy constructor` (see below) as it can't know if a
 * copy is valid. For the auto command system, we require a duplicate() method
 * in AutoCommandInterface that safely duplicates the initial state of a command
 * so AutoCommand can have safe, effecient copy operations while having a
 * simple, easy to use, hard to misuse interface.
 *
 * RAII requires some special member functions to tell the compiler what it
 * should do.
 * - Copy constructor - Initialize an object by copying the resources of another
 * (For AutoCommand, duplicate())
 * - Move constructor - Initialize an object by taking ownership of the
 * resources (more effecient but it leaves the other one in an uninitialized
 * state)
 * - Copy assignment - Set this object equal to the state of another without
 * modifying the original. (For AutoCommand, duplicate())
 * - Move assignment - Set this object equal to the state of another by taking
 * ownership of the other
 * - Destructor - What to do when no one needs this object anymore
 */

/**
 * Copy Constructor
 */
AutoCommand::AutoCommand(const AutoCommand &other) {
    *this = other.cmd_ptr->duplicate();
}
/**
 * Move Constructor
 */
AutoCommand::AutoCommand(AutoCommand &&other)
    : cmd_ptr(other.cmd_ptr), timeout_seconds(other.timeout_seconds) {
    // we took ownership of the command, other now loses ownership
    other.cmd_ptr = nullptr;
}

/**
 * Copy Assignment
 */
AutoCommand AutoCommand::operator=(const AutoCommand &other) {
    return AutoCommand(other);
}
/**
 * Move Assignment
 */

AutoCommand AutoCommand::operator=(AutoCommand &&other) {
    return AutoCommand(other);
}

/**
 * Destructor
 * Free the memory we allocated to hold the AutoCommandInterface object
 */
AutoCommand::~AutoCommand() {
    if (cmd_ptr == nullptr) {
        return;
    }
    delete cmd_ptr;
}

InOrder::InOrder() : cmds{} {}
InOrder::InOrder(std::initializer_list<AutoCommand> cmds) : cmds{cmds} {}

InOrder InOrder::FromVector(const std::vector<AutoCommand> &cmds) {
    InOrder i;
    i.cmds = cmds;
    return i;
}

bool InOrder::run() {
    if (cmd_index == -1) {
        // Just starting
        command_timer.reset();
        cmd_index = 0;
    } else if (cmd_index >= cmds.size()) {
        // Finished
        return true;
    }
    // check for cmd's timeout
    AutoCommand &cmd = cmds[cmd_index];
    if (cmd.does_timeout() && command_timer.value() > cmd.timeout_time()) {
        cmd.on_timeout();
        cmd_index++;
        command_timer.reset();
        return true;
    }
    // check for cmd's condition
    if (cmd.true_to_end->test()) {
        cmd.on_timeout();
        cmd_index++;
        command_timer.reset();
        return true;
    }
    // actually act on the command
    const bool command_ended_normally = cmd.run();
    if (command_ended_normally) {
        cmd_index++;
        command_timer.reset();
        if (cmd_index >= cmds.size()) {
            return true;
        }
    }

    // still more work to do
    return false;
}

AutoCommand InOrder::duplicate() const {
    InOrder other;
    other.cmds = cmds;
    return other.with_timeout(AutoCommand::DONT_TIMEOUT);
}
AutoCommand InOrder::with_timeout(double seconds) {
    return AutoCommand(*this).with_timeout(seconds);
}

AutoCommand InOrder::until(Condition cond) {
    return AutoCommand(*this).until(std::forward<Condition>(cond));
}

InOrder InOrder::repeat_times(size_t N) {
    cmds.reserve(N * cmds.size());
    size_t og_size = cmds.size();
    // copy over commands N times
    for (size_t i = 0; i < N - 1; i++) {
        for (size_t j = 0; j < og_size; j++) {
            cmds.push_back(cmds[j]);
        }
    }
    return *this;
}

Repeat::Repeat() : cmds{} {}
Repeat::Repeat(std::initializer_list<AutoCommand> cmds)
    : cmds{cmds}, working_cmds{cmds} {}

Repeat Repeat::FromVector(const std::vector<AutoCommand> &cmds) {
    Repeat r;
    r.cmds = cmds;
    r.working_cmds = InOrder::FromVector(cmds);
    return r;
}

bool Repeat::run() {
    bool inorder_ended = working_cmds.run();
    if (inorder_ended) {
        working_cmds = InOrder::FromVector(cmds);
    }
    return false;
}

AutoCommand Repeat::duplicate() const { return Repeat::FromVector(cmds); }
AutoCommand Repeat::with_timeout(double seconds) {
    return AutoCommand(*this).with_timeout(seconds);
}

AutoCommand Repeat::until(Condition cond) {
    return AutoCommand(*this).until(std::forward<Condition>(cond));
}

Condition TimeSinceStartExceeds(double seconds) {
    // tmr is started at path creation time. which we say equals the start of
    // the path (true most of the time)
    return fc(
        [seconds, tmr = vex::timer()]() { return tmr.value() > seconds; });
}

Message::Message(const std::string &msg) : msg(msg) {}

bool Message::run() {
    printf("%s", msg.c_str());
    return true;
}
AutoCommand Message::duplicate() const { return Message(msg); }

Branch::Branch(Condition decider, AutoCommand iftrue, AutoCommand iffalse)
    : is_decided(false), decider(std::move(decider)), iftrue(std::move(iftrue)),
      iffalse(std::move(iffalse)) {}
bool Branch::run() {
    if (!is_decided) {
        choice = decider->test();
        is_decided = true;
    }
    if (choice) {
        return iftrue.run();
    }
    return iffalse.run();
}
AutoCommand Branch::duplicate() const {
    return Branch(decider, iftrue, iffalse);
}

FunctionCommand::FunctionCommand(std::function<bool()> f) : f(std::move(f)) {}

AutoCommand FunctionCommand::duplicate() const { return FunctionCommand(f); }
