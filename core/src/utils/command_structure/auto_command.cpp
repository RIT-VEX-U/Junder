#include "../core/include/utils/command_structure/auto_command.h"

InOrder::InOrder(std::queue<AutoCommand *> cmds) : cmds(cmds)
{
    timeout_seconds = -1.0; // never timeout unless with_timeout is explicitly called
}
bool InOrder::run()
{
    // outer loop finished
    if (cmds.size() == 0)
    {
        return true;
    }
    // retrieve and remove command at the front of the queue
    if (current_command == nullptr)
    {
        current_command = cmds.front();
        cmds.pop();
        tmr.reset();
    }

    // run command
    bool cmd_finished = current_command->run();
    if (cmd_finished)
    {
        current_command = nullptr;
        return false; // continue omn next command
    }

    double seconds = static_cast<double>(tmr.time()) / 1000.0;

    bool doTimeout = current_command->timeout_seconds > 0.0;
    bool command_timed_out = seconds > current_command->timeout_seconds;

    // timeout
    if (doTimeout && command_timed_out)
    {
        current_command->on_timeout();
        current_command = nullptr;
        return false;
    }
    return false;
}

void InOrder::on_timeout()
{
    if (current_command != nullptr)
    {
        current_command->on_timeout();
    }
}

FirstFinish::FirstFinish(std::vector<AutoCommand *> cmds) : cmds(cmds), tmr()
{
    timeout_seconds = -1.0; // dont timeout `unless explicitly told to
}
bool FirstFinish::run()
{
    for (size_t i = 0; i < cmds.size(); i++)
    {
        bool finished = cmds[i]->run();
        bool doTimeout = cmds[i]->timeout_seconds > 0.0;

        double seconds = static_cast<double>(tmr.time()) / 1000.0;
        bool timed_out = seconds > cmds[i]->timeout_seconds;
        if ((doTimeout && timed_out) || finished)
        {
            finished_idx = i;
            break;
        }
    }
    if (finished_idx != -1)
    {
        for (int i = 0; i < cmds.size(); i++)
        {
            if (i == finished_idx)
            {
                continue;
            }
            cmds[i]->on_timeout();
        }
        return true;
    }
    return false;
}
void FirstFinish::on_timeout()
{
    for (int i = 0; i < cmds.size(); i++)
    {
        if (i == finished_idx)
        {
            continue;
        }

        cmds[i]->on_timeout();
    }
}

// wait for all to finish
Parallel::Parallel(std::vector<AutoCommand *> cmds) : cmds(cmds), finished(cmds.size(), false)
{
    timeout_seconds = -1;
}
bool Parallel::run()
{
    bool all_finished = true;

    for (int i = 0; i < cmds.size(); i++)
    {
        if (finished[i] == true)
        {
            continue;
        }
        finished[i] = cmds[i]->run();
        if (!finished[i])
        {
            all_finished = false;
        }
    }
    return all_finished;
}
void Parallel::on_timeout()
{
    for (int i = 0; i < finished.size(); i++)
    {
        if (!finished[i])
        {
            cmds[i]->on_timeout();
        }
    }
}

Branch::Branch(AutoCommand *false_choice, AutoCommand *true_choice, Predicate *pred) : false_choice(false_choice), true_choice(true_choice), pred(pred), choice(false), chosen(false), tmr() {}
bool Branch::run()
{
    if (!chosen)
    {
        choice = pred->test();
        chosen = true;
    }

    double seconds = static_cast<double>(tmr.time()) / 1000.0;
    if (choice == false)
    {
        if (seconds > false_choice->timeout_seconds && false_choice->timeout_seconds != -1)
        {
            false_choice->on_timeout();
        }
        return false_choice->run();
    }
    else
    {
        if (seconds > true_choice->timeout_seconds && true_choice->timeout_seconds != -1)
        {
            true_choice->on_timeout();
        }
        return true_choice->run();
    }
}
void Branch::on_timeout()
{
    if (!chosen)
    {
        // dont need to do anything
        return;
    }

    if (choice == false)
    {
        false_choice->on_timeout();
    }
    else
    {
        true_choice->on_timeout();
    }
}
