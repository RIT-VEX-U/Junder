
#pragma once
#include "vex.h"
#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/screen.h"

class CataSys
{
public:
    enum class Command
    {
        IntakeIn,    // all mutually exclusive or else we get DQed or jam the cata
        IntakeHold,  // all mutually exclusive or else we get DQed or jam the cata
        StartFiring, // all mutually exclusive or else we get DQed or jam the cata
        StopFiring,
        StopIntake,
        IntakeOut,
    };
    enum class IntakeType
    {
        In,
        Out,
        Hold,
    };

    struct State
    {
        bool ball_in_intake;
        bool ball_in_cata;
        bool cata_in_position;
    };

    CataSys(vex::optical &intake_watcher, vex::pot &cata_pot, vex::optical &cata_watcher, vex::motor_group &cata_motor, vex::motor &intake_upper, vex::motor &intake_lower);
    void send_command(Command cmd);
    State get_state() const;

    // Autocommands
    AutoCommand *Fire();
    AutoCommand *IntakeToHold();
    AutoCommand *IntakeFully();

    // Page
    screen::Page *Page();

private:
    // configuration
    vex::optical &intake_watcher;
    vex::pot &cata_pot;
    vex::optical &cata_watcher;
    vex::motor_group &cata_motor;
    vex::motor &intake_upper;
    vex::motor &intake_lower;

    // running
    vex::task runner;
    vex::mutex control_mut;
    // THESE SHOULD ONLY BE ACCESSED BEHIND THE MUTEX
    State state;
    bool firing_requested;
    bool intaking_requested;
    IntakeType intake_type;
    friend int thread_func(void *void_cata);
};