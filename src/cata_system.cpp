#include "cata_system.h"
#include <string>

// const double inake_enable_lower_threshold = 70.0;
// const double intake_enable_upper_threshold = 120;
const double inake_enable_lower_threshold = 150;
const double intake_enable_upper_threshold = 200;

const double intake_upper_outer_volt = 12.0;
const double intake_lower_outer_volt = 9.0;
const double intake_upper_volt = 12;
const double intake_upper_volt_hold = 6;
const double intake_lower_volt = 9.0;
const double intake_sensor_dist_mm = 150;

const double cata_target_charge = 179;
const double cata_target_intake = 179; // LOWER IS CLOSER TO SLIPPPING

const double intake_drop_seconds = 0.5;
const double fire_time = 0.4;
const double fire_voltage = 12.0;
;

bool intake_can_be_enabled(double cata_pos) {
    return (cata_pos == 0.0) || (cata_pos > inake_enable_lower_threshold &&
                                 cata_pos < intake_enable_upper_threshold);
}
std::string to_string(CataOnlyState s) {
    switch (s) {
    case CataOnlyState::Starting:
        return "Starting";
    case CataOnlyState::Firing:
        return "Firing";
    case CataOnlyState::Reloading:
        return "Reloading";
    case CataOnlyState::ReadyToFire:
        return "ReadyToFire";
    }
    return "";
}

std::string to_string(CataOnlyMessage m) {
    switch (m) {
    case CataOnlyMessage::DoneReloading:
        return "Done Reloading";
    case CataOnlyMessage::DoneFiring:
        return "Done Firing";
    case CataOnlyMessage::Fire:
        return "Fire";
    }
    return "";
}

struct Reloading : public CataOnlySys::State {
    void entry(CataOnlySys &sys) override {
        sys.pid.set_target(cata_target_charge);
    }

    CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
        // work on motor
        double cata_deg = sys.pot.angle();
        sys.pid.update(cata_deg);
        sys.mot.spin(vex::fwd, sys.pid.get(), vex::volt);
        // are we there yettt
        if (sys.pid.is_on_target()) {
            return CataOnlyMessage::DoneReloading;
        }
        // otherwise keep chugging
        return {};
    }

    CataOnlyState id() const override { return CataOnlyState::Reloading; }
    State *respond(CataOnlyMessage m) override;
};

class Firing : public CataOnlySys::State {
  public:
    void entry(CataOnlySys &sys) override {
        tmr.reset();
        sys.mot.spin(vex::fwd, fire_voltage, vex::volt);
    }
    CataOnlySys::MaybeMessage work(CataOnlySys &sys) {
        if (tmr.value() > fire_time) {
            return CataOnlyMessage::DoneFiring;
        }
        return {};
    }
    CataOnlyState id() const override { return CataOnlyState::Firing; }
    State *respond(CataOnlyMessage m) override;

  private:
    vex::timer tmr;
};

class ReadyToFire : public CataOnlySys::State {
  public:
    CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
        double cata_deg = sys.pot.angle(vex::degrees);
        sys.pid.update(cata_deg);
        sys.mot.spin(vex::fwd, sys.pid.get(), vex::volt);

        // hold here until message comes from outside
        return {};
    }
    State *respond(CataOnlyMessage m) override;
    CataOnlyState id() const override { return CataOnlyState::ReadyToFire; }
};
CataOnlySys::State *Reloading::respond(CataOnlyMessage m) {
    if (m == CataOnlyMessage::DoneReloading) {
        return new ReadyToFire();
    }
    // Ignore other messages
    return this;
}
CataOnlySys::State *ReadyToFire::respond(CataOnlyMessage m) {
    if (m == CataOnlyMessage::Fire) {
        return new Firing();
    }
    // Ignore other messages

    return this;
}

CataOnlySys::State *Firing::respond(CataOnlyMessage m) {
    if (m == CataOnlyMessage::DoneFiring) {
        return new Reloading();
    }
    // Ignore other messages
    return this;
}
// int thread_func(void *void_cata) {
//     CataSys &cata = *(CataSys *)void_cata;
//     cata.state = CataSys::CataState::CHARGING;

//     vex::timer intake_tmr;
//     cata_pid.set_limits(-12.0, 0);

//     while (true) {
//         if (cata.state == CataSys::CataState::UNFOLDING) {
//             if (cata.drop_timer.value() < intake_drop_seconds) {
//                 cata.intake_lower.spin(vex::reverse, 12.0,
//                                        vex::voltageUnits::volt);
//             } else {
//                 cata.state = CataSys::CataState::CHARGING;
//             }
//             continue;
//         }
//         // read sensors
//         const double cata_pos = cata.cata_pot.angle(vex::degrees);

//         if (cata.intake_watcher.objectDistance(distanceUnits::mm) <
//             intake_sensor_dist_mm) {
//             intake_tmr.reset();
//         }

//         const bool ball_in_intake =
//             cata.intake_watcher.objectDistance(distanceUnits::mm) <
//                 intake_sensor_dist_mm ||
//             intake_tmr.time(timeUnits::msec) < 500;

//         const bool ball_in_cata = cata.cata_watcher.isNearObject();

//         // SYNCHRONIZE (READ)
//         cata.control_mut.lock();
//         CataSys::CataState cur_state = cata.state;
//         bool firing_requested = cata.firing_requested;
//         bool intaking_requested = cata.intaking_requested;
//         bool matchload_requested = cata.matchload_requested;
//         CataSys::IntakeType intake_type = cata.intake_type;
//         cata.control_mut.unlock();

//         bool intake_cata_enabled = false;
//         double cata_pid_out = 0;

//         // Main Intake State Machine
//         switch (cur_state) {
//         case CataSys::CataState::FIRING:
//             cata.intake_upper.stop();
//             cata.intake_lower.stop();
//             break;
//         case CataSys::CataState::CHARGING:
//         case CataSys::CataState::READY:
//             // Charging & Ready: Base off of cata position
//             // ==== INTAKE ====
//             // Run based on requested action
//             // Make sure the cata is in a good position

//             if (intaking_requested && intake_can_be_enabled(cata_pos) &&
//                 intake_type == CataSys::IntakeType::In && !ball_in_cata) {
//                 // Intake triball
//                 cata.intake_upper.spin(vex::fwd, intake_upper_volt,
//                 vex::volt); cata.intake_lower.spin(vex::fwd,
//                 intake_lower_volt, vex::volt);
//             } else if (intaking_requested && intake_can_be_enabled(cata_pos)
//             &&
//                        intake_type == CataSys::IntakeType::Hold &&
//                        !ball_in_cata) {
//                 // Intake until ball is sensed in intake, then stop
//                 // timer to make sure it doesn't go straight through
//                 if (ball_in_intake) {
//                     cata.intake_upper.stop(brakeType::hold);
//                     cata.intake_lower.stop(brakeType::hold);
//                 } else {
//                     cata.intake_upper.spin(vex::fwd, intake_upper_volt_hold,
//                                            vex::volt);
//                     cata.intake_lower.spin(vex::fwd, intake_lower_volt,
//                                            vex::volt);
//                 }
//             } else if (intaking_requested &&
//                        intake_type == CataSys::IntakeType::Out) {
//                 cata.intake_upper.spin(vex::fwd, -intake_upper_outer_volt,
//                                        vex::volt);
//                 cata.intake_lower.spin(vex::fwd, -intake_lower_outer_volt,
//                                        vex::volt);
//             } else if (intaking_requested &&
//                        intake_type == CataSys::IntakeType::JustOut) {
//                 if (ball_in_intake) {
//                     cata.intake_upper.spin(vex::fwd, -6.0, vex::volt);
//                     cata.intake_lower.spin(vex::fwd, -6.0, vex::volt);
//                 } else {
//                     cata.intake_upper.stop(brakeType::coast);
//                     cata.intake_lower.stop(brakeType::coast);
//                 }
//             } else {
//                 cata.intake_upper.stop();
//                 cata.intake_lower.stop();
//             }

//             // Reset requests from AutoCommands
//             if ((intake_type == CataSys::IntakeType::Hold && ball_in_intake)
//             ||
//                 (intake_type == CataSys::IntakeType::In && ball_in_cata)) {
//                 // intaking_requested = false;
//             }
//         }

//         // Main catapult state machine
//         switch (cur_state) {
//         case CataSys::CataState::CHARGING:

//             // ==== CATAPULT ===
//             // Run via PID
//             cata_pid.set_target(cata_target_charge);
//             cata_pid_out = cata_pid.update(cata_pos);
//             cata.cata_motor.spin(vex::fwd, cata_pid_out, vex::volt);

//             // ==== EXIT STATE ====
//             // Ratchet engaged, we are READY
//             if (cata_pid.is_on_target() ||
//                 cata_pos < cata_target_charge - pc.deadband) {
//                 cur_state = CataSys::CataState::READY;
//             }
//             break;
//         case CataSys::CataState::READY:

//             // ==== CATAPULT ====
//             // Disable, rely on ratchet

//             if (intake_cata_enabled == false) {
//                 // cata.cata_motor.stop(brakeType::coast);

//                 // REMOVE IF RATCHETING
//                 cata_pid.set_target(cata_target_charge);
//                 cata_pid_out = cata_pid.update(cata_pos);
//                 cata.cata_motor.spin(vex::fwd, cata_pid_out, vex::volt);
//             }

//             // When firing is requested, FIRE!
//             if (firing_requested && cata.can_fire()) {
//                 cur_state = CataSys::CataState::FIRING;
//             }

//             // Check if position is too high, go back to charging.
//             if (cata_pos > cata_target_charge + 30) {
//                 cur_state = CataSys::CataState::CHARGING;
//             }
//             break;
//         case CataSys::CataState::FIRING:

//             // ==== CATAPULT ====
//             cata.cata_motor.spinFor(directionType::rev, 500, timeUnits::msec,
//                                     100.0, velocityUnits::pct);
//             cata_pid.reset(); // reset integral

//             if (cata.cata_motor.isDone()) {
//                 cur_state = CataSys::CataState::CHARGING;
//                 firing_requested = false;
//             }
//             break;
//         }

//         if (cata.cata_watcher.isNearObject() &&
//             cata.intake_watcher.objectDistance(distanceUnits::mm) < 150) {
//             printf(
//                 "DQed! ball in intake and catapault (or a sensor got
//                 funny)\n");
//         }

//         // SYNCHRONIZE
//         cata.control_mut.lock();
//         cata.state = cur_state;
//         cata.firing_requested = firing_requested;
//         cata.intaking_requested = intaking_requested;
//         cata.control_mut.unlock();

//         vexDelay(5);
//     }
//     return 0;
// }

CataSys::CataSys(vex::distance &intake_watcher, vex::pot &cata_pot,
                 vex::optical &cata_watcher, vex::motor_group &cata_motor,
                 vex::motor &intake_upper, vex::motor &intake_lower,
                 PIDFF cata_feedback)
    : intake_watcher(intake_watcher), cata_pot(cata_pot),
      cata_watcher(cata_watcher), cata_motor(cata_motor),
      intake_upper(intake_upper), intake_lower(intake_lower),
      cata_sys(cata_pot, cata_watcher, cata_motor, cata_feedback) {}

void CataSys::send_command(Command next_cmd) {
    // control_mut.lock();
    switch (next_cmd) {
    case CataSys::Command::StartFiring:
        // firing_requested = true;
        break;
    case CataSys::Command::StopFiring:
        // firing_requested = false;
        break;
    case CataSys::Command::IntakeIn:
        // intaking_requested = true;
        // intake_type = CataSys::IntakeType::In;
        break;
    case CataSys::Command::IntakeOut:
        // intaking_requested = true;
        // intake_type = CataSys::IntakeType::Out;
        break;
    case CataSys::Command::OuttakeJust:
        // intaking_requested = true;
        // intake_type = CataSys::IntakeType::JustOut;
        break;
        //
    case CataSys::Command::IntakeHold: // not handled
        // intaking_requested = true;
        // intake_type = CataSys::IntakeType::Hold;
        break;
    case CataSys::Command::StopIntake:
        // intaking_requested = false;
        break;
    case CataSys::Command::StartMatchLoad:
        // matchload_requested = true;
        break;
    case CataSys::Command::StopMatchLoad:
        // matchload_requested = false;
        break;
    case CataSys::Command::StartDropping:
        // drop_timer.reset();
        // state = CataState::UNFOLDING;
        break;
    default:
        break;
    }
    // control_mut.unlock();
}

// CataSys::CataState CataSys::get_state() const {
//     control_mut.lock();
//     auto s = state;
//     control_mut.unlock();
//     return s;
// }

// bool CataSys::can_fire() const {
//     return get_state() == CataState::READY; // cata_watcher.isNearObject() &&
// }

class CataSysPage : public screen::Page {
  public:
    CataSysPage(const CataSys &cs)
        : gd(30, 130.0, 270.0, {vex::green, vex::red}, 2), cs(cs) {}
    void update(bool, int, int) override {}

    void draw(vex::brain::lcd &scr, bool, unsigned int) override {
        // CataSys::CataState state = cs.get_state();
        // const char *state_str = "UNKNOWN STATE";
        // switch (state) {
        // case CataSys::CataState::CHARGING:
        //     state_str = "CHARGING";
        //     break;
        // case CataSys::CataState::FIRING:
        //     state_str = "FIRING";
        //     break;
        // case CataSys::CataState::READY:
        //     state_str = "READY";
        //     break;

        // default:
        //     break;
        // }

        // gd.add_samples({cata_pid.get_sensor_val(), cata_pid.get_target()});

        // const bool ball_in_intake =
        //     cs.intake_watcher.objectDistance(distanceUnits::mm) <
        //     intake_sensor_dist_mm;

        // const bool ball_in_cata = cs.cata_watcher.isNearObject();

        // const char *intake_dir = "";
        // if (cs.intaking_requested) {
        //     switch (cs.intake_type) {
        //     case CataSys::IntakeType::Hold:
        //         intake_dir = "Hold";
        //         break;
        //     case CataSys::IntakeType::In:
        //         intake_dir = "In";
        //         break;
        //     case CataSys::IntakeType::Out:
        //         intake_dir = "Out";
        //         break;
        //     case CataSys::IntakeType::JustOut:
        //         intake_dir = "JustOut";
        //         break;
        //     }
        // }

        // scr.printAt(40, 20, true, "Cata: %s", state_str);
        // scr.printAt(40, 60, true, "MatchLoad Requested: %s",
        //             cs.matchload_requested ? "yes" : "no");
        // scr.printAt(40, 80, true, "Intake Requested: %s%s",
        //             cs.intaking_requested ? "yes" : "no", intake_dir);
        // scr.printAt(40, 100, true, "Fire Requested: %s",
        //             cs.firing_requested ? "yes" : "no");

        // scr.printAt(40, 140, true, "Ball in Cata: %s",
        //             ball_in_cata ? "yes" : "no");

        // scr.printAt(40, 160, true, "Ball in Intake: %s",
        //             ball_in_intake ? "yes" : "no");
        // scr.printAt(40, 180, true, "Can Fire: %s",
        //             cs.can_fire() ? "yes" : "no");

        // gd.draw(scr, 240, 0, 200, 200);
    }

  private:
    GraphDrawer gd;
    const CataSys &cs;
};

screen::Page *CataSys::Page() { return new CataSysPage(*this); }

AutoCommand *CataSys::StopIntake() {
    return new FunctionCommand([&]() {
        send_command(Command::StopIntake);
        return true;
    });
}

AutoCommand *CataSys::Fire() {
    return new FunctionCommand([&]() {
        send_command(Command::StartFiring);
        return true;
    });
}

AutoCommand *CataSys::IntakeFully() {
    return new FunctionCommand([&]() {
        send_command(Command::IntakeIn);
        return true;
    });
}

AutoCommand *CataSys::IntakeToHold() {
    return new FunctionCommand([&]() {
        send_command(Command::IntakeHold);
        return true;
    });
}

AutoCommand *CataSys::WaitForIntake() {
    return new FunctionCommand([&]() { return cata_watcher.isNearObject(); });
}

AutoCommand *CataSys::WaitForHold() {
    return new FunctionCommand([&]() {
        return intake_watcher.objectDistance(distanceUnits::mm) < 150;
    });
}
AutoCommand *CataSys::StopFiring() {
    return new FunctionCommand([&]() {
        send_command(CataSys::Command::StopFiring);
        return true;
    });
}

AutoCommand *CataSys::Unintake() {
    return new FunctionCommand([&]() {
        send_command(CataSys::Command::IntakeOut);
        return true;
    });
}
