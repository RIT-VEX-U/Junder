
#pragma once
#include "core/subsystems/custom_encoder.h"
#include "core/subsystems/screen.h"
#include "core/utils/command_structure/auto_command.h"
#include "vex.h"

class CataSys {
  public:
    enum class Command {
        IntakeIn, // all mutually exclusive or else we get DQed or jam the cata
        IntakeHold,  // all mutually exclusive or else we get DQed or jam the
                     // cata
        StartFiring, // all mutually exclusive or else we get DQed or jam the
                     // cata
        StopFiring,
        StopIntake,
        IntakeOut,
        StartMatchLoad,
        StopMatchLoad
    };
    enum class IntakeType {
        In,
        Out,
        Hold,
    };

    enum CataState { CHARGING, READY, FIRING };

    CataSys(vex::distance &intake_watcher, vex::pot &cata_pot,
            vex::optical &cata_watcher, vex::motor_group &cata_motor,
            vex::motor &intake_upper, vex::motor &intake_lower);
    void send_command(Command cmd);
    CataState get_state() const;
    bool can_fire() const;

    // Autocommands
    AutoCommand Fire();
    AutoCommand StopFiring();
    AutoCommand StopIntake();
    AutoCommand IntakeToHold();
    AutoCommand IntakeFully();
    AutoCommand WaitForIntake();

    // Page
    screen::Page *Page();

  private:
    // configuration
    vex::distance &intake_watcher;
    vex::pot &cata_pot;
    vex::optical &cata_watcher;
    vex::motor_group &cata_motor;
    vex::motor &intake_upper;
    vex::motor &intake_lower;

    // running
    vex::task runner;
    mutable vex::mutex
        control_mut; // I am sorry for my crimes. However, get_state() needs to
                     // lock this but is conceptually constant.
    // THESE SHOULD ONLY BE ACCESSED BEHIND THE MUTEX
    CataState state;
    bool firing_requested;
    bool intaking_requested;
    bool matchload_requested;
    IntakeType intake_type;
    friend int thread_func(void *void_cata);
    friend class CataSysPage;
};