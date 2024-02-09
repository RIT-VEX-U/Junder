
#pragma once
#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/subsystems/screen.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/utils/state_machine.h"
#include "vex.h"

enum class CataOnlyMessage { DoneReloading, DoneFiring, Fire, Slipped };
enum class CataOnlyState { Firing, Reloading, ReadyToFire };

class CataOnlySys : public StateMachine<CataOnlySys, CataOnlyState,
                                        CataOnlyMessage, 5, true> {
  public:
    friend struct Reloading;
    friend class Firing;
    friend class ReadyToFire;
    friend class CataSysPage;
    CataOnlySys(vex::pot &cata_pot, vex::optical &cata_watcher,
                vex::motor_group &cata_motor, PIDFF &cata_pid);

  private:
    vex::pot &pot;
    vex::optical &cata_watcher;
    vex::motor_group &mot;
    PIDFF &pid;
};

class CataSys {
  public:
    enum class Command {
        IntakeIn, // all mutually exclusive or else we get DQed or jam the cata
        IntakeHold,  // all mutually exclusive or else we get DQed or jam the
                     // cata
        StartFiring, // all mutually exclusive or else we get DQed or jam the
                     // cata
        StopIntake,
        IntakeOut,
        StartMatchLoad,
        StopMatchLoad,
        StartDropping,
        OuttakeJust
    };
    enum class IntakeType {
        In,
        Out,
        Hold,
        JustOut,
    };

    enum CataState { CHARGING, READY, FIRING, UNFOLDING };

    CataSys(vex::distance &intake_watcher, vex::pot &cata_pot,
            vex::optical &cata_watcher, vex::motor_group &cata_motor,
            vex::motor &intake_upper, vex::motor &intake_lower,
            PIDFF &cata_feedback);
    void send_command(Command cmd);
    CataState get_state() const;
    bool can_fire() const;

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

    friend class CataSysPage;
};