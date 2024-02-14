
#pragma once
#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/subsystems/screen.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/utils/state_machine.h"
#include "cata/cata.h"
#include "cata/intake.h"
#include "vex.h"

class CataSys {
  public:
    enum class Command {
        IntakeIn,
        IntakeHold,
        StopIntake,
        IntakeOut,
        StartDropping,
        StartFiring,
        ToggleCata,
    };

    CataSys(vex::distance &intake_watcher, vex::pot &cata_pot,
            vex::optical &cata_watcher, vex::motor_group &cata_motor,
            vex::motor &intake_upper, vex::motor &intake_lower,
            PIDFF &cata_feedback, DropMode drop);
    void send_command(Command cmd);
    bool can_fire() const;
    // Returns true when the cata system is finished dropping
    bool still_dropping();

    // Autocommands
    AutoCommand *Fire();
    AutoCommand *StopIntake();
    AutoCommand *IntakeToHold();
    AutoCommand *IntakeFully();
    AutoCommand *WaitForIntake();
    AutoCommand *WaitForHold();
    AutoCommand *Unintake();

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
    CataOnlySys cata_sys;
    IntakeSys intake_sys;
    friend class CataSysPage;
};