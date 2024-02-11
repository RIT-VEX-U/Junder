#include "cata_system.h"
#include <string>

const double inake_enable_lower_threshold = 150;
const double intake_enable_upper_threshold = 200;

const double intake_upper_outer_volt = 12.0;
const double intake_lower_outer_volt = 9.0;

const double intake_upper_volt = 12;
const double intake_lower_volt = 9.0;

const double intake_upper_volt_hold = 12.0;
const double intake_lower_volt_hold = 9.0;

const double intake_sensor_dist_mm = 150;

const double cata_target_charge = 180;
const double done_firing_angle = 200;

const double intake_drop_seconds = 0.5;
const double intake_drop_seconds_until_enable = 0.25;
const double fire_voltage = 12.0;

bool intake_can_be_enabled(double cata_pos) {
    return (cata_pos == 0.0) || (cata_pos > inake_enable_lower_threshold &&
                                 cata_pos < intake_enable_upper_threshold);
}

// Cata
// ==============================================================================================================================

class CataOff : public CataOnlySys::State {
  public:
    void entry(CataOnlySys &sys) override {
        sys.mot.stop(vex::brakeType::coast);
    }
    CataOnlyState id() const override { return CataOnlyState::CataOff; }
    State *respond(CataOnlySys &sys, CataOnlyMessage m) override;
};

class WaitingForDrop : public CataOnlySys::State {
  public:
    void entry(CataOnlySys &sys) override { drop_timer.reset(); }
    CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
        if (drop_timer.value() > intake_drop_seconds_until_enable) {
            return CataOnlyMessage::EnableCata;
        }
        return {};
    }
    CataOnlyState id() const override { return CataOnlyState::WaitingForDrop; }
    State *respond(CataOnlySys &sys, CataOnlyMessage m) override;

  private:
    vex::timer drop_timer;
};

struct Reloading : public CataOnlySys::State {
    void entry(CataOnlySys &sys) override {

        sys.pid.update(sys.pot.angle(vex::deg));
        sys.pid.set_target(cata_target_charge);
    }

    CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
        // work on motor
        double cata_deg = sys.pot.angle(vex::deg);
        if (cata_deg == 0.0) {
            // adc hasnt warmed up yet, we're getting silly results
            return {};
        }
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
    State *respond(CataOnlySys &sys, CataOnlyMessage m) override;
};

class Firing : public CataOnlySys::State {
  public:
    void entry(CataOnlySys &sys) override {
        sys.mot.spin(vex::reverse, fire_voltage, vex::volt);
    }
    CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
        // started goin up again
        if (sys.pot.angle(vex::deg) > done_firing_angle) {
            return CataOnlyMessage::DoneFiring;
        }
        return {};
    }
    CataOnlyState id() const override { return CataOnlyState::Firing; }
    State *respond(CataOnlySys &sys, CataOnlyMessage m) override;
};

class ReadyToFire : public CataOnlySys::State {
  public:
    CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
        double cata_deg = sys.pot.angle(vex::degrees);
        sys.pid.update(cata_deg);
        sys.mot.spin(vex::fwd, sys.pid.get(), vex::volt);

        // If we slipped, send message to go back to reload
        if (!intake_can_be_enabled(cata_deg)) {
            printf("Slipped\n");
            fflush(stdout);
            return CataOnlyMessage::Slipped;
        }

        // hold here until message comes from outside
        return {};
    }
    State *respond(CataOnlySys &sys, CataOnlyMessage m) override;
    CataOnlyState id() const override { return CataOnlyState::ReadyToFire; }
};

CataOnlySys::State *CataOff::respond(CataOnlySys &sys, CataOnlyMessage m) {
    if (m == CataOnlyMessage::EnableCata) {
        return new Reloading();
    } else if (m == CataOnlyMessage::StartDrop) {
        return new WaitingForDrop();
    }
    // Ignore other messages
    return this;
}

CataOnlySys::State *WaitingForDrop::respond(CataOnlySys &sys,
                                            CataOnlyMessage m) {
    if (m == CataOnlyMessage::EnableCata) {
        return new Reloading();
    }
    // Ignore other messages
    return this;
}

CataOnlySys::State *Reloading::respond(CataOnlySys &sys, CataOnlyMessage m) {
    if (m == CataOnlyMessage::DoneReloading) {
        return new ReadyToFire();
    } else if (m == CataOnlyMessage::DisableCata) {
        return new CataOff();
    }
    // Ignore other messages
    return this;
}
CataOnlySys::State *ReadyToFire::respond(CataOnlySys &sys, CataOnlyMessage m) {
    if (m == CataOnlyMessage::Fire) {
        return new Firing();
    } else if (m == CataOnlyMessage::Slipped) {
        return new Reloading();
    } else if (m == CataOnlyMessage::DisableCata) {
        return new CataOff();
    }

    // Ignore other messages
    return this;
}

CataOnlySys::State *Firing::respond(CataOnlySys &sys, CataOnlyMessage m) {
    if (m == CataOnlyMessage::DoneFiring) {
        return new Reloading();
    } else if (m == CataOnlyMessage::DisableCata) {
        return new CataOff();
    }
    // Ignore other messages
    return this;
}
std::string to_string(CataOnlyState s) {
    switch (s) {
    case CataOnlyState::Firing:
        return "Firing";
    case CataOnlyState::Reloading:
        return "Reloading";
    case CataOnlyState::ReadyToFire:
        return "ReadyToFire";
    case CataOnlyState::CataOff:
        return "CataOff";
    case CataOnlyState::WaitingForDrop:
        return "WaitingForDrop";
    }
    return "UNHANDLED CATA STATE";
}

std::string to_string(CataOnlyMessage m) {
    switch (m) {
    case CataOnlyMessage::Slipped:
        return "Slipped";
    case CataOnlyMessage::DoneReloading:
        return "Done Reloading";
    case CataOnlyMessage::DoneFiring:
        return "Done Firing";
    case CataOnlyMessage::Fire:
        return "Fire";
    case CataOnlyMessage::EnableCata:
        return "EnableCata";
    case CataOnlyMessage::DisableCata:
        return "DisableCata";
    case CataOnlyMessage::StartDrop:
        return "StartDrop";
    }
    return "UNHANDLED CATA MESSAGE";
}

// INTAKE
// ==============================================================================================================================
bool CataOnlySys::intaking_allowed() {
    return current_state() == CataOnlyState::ReadyToFire &&
           !cata_watcher.isNearObject();
}

bool IntakeSys::ball_in_intake() {
    return intake_watcher.objectDistance(distanceUnits::mm) <
           intake_sensor_dist_mm;
}
std::string to_string(IntakeState s) {
    switch (s) {
    case IntakeState::Dropping:
        return "Dropping";
    case IntakeState::Intaking:
        return "Intaking";
    case IntakeState::IntakingHold:
        return "Intaking to hold";
    case IntakeState::Outtaking:
        return "Outtaking";
    case IntakeState::Stopped:
        return "Stopped";
    }
    return "UNKNOWN INTAKE STATE";
}

std::string to_string(IntakeMessage m) {
    switch (m) {
    case IntakeMessage::Dropped:
        return "Dropped";
    case IntakeMessage::Intake:
        return "Intake";
    case IntakeMessage::IntakeHold:
        return "IntakeToHold";
    case IntakeMessage::Outtake:
        return "Outtake";
    case IntakeMessage::StopIntake:
        return "Stop";
    case IntakeMessage::Drop:
        return "Drop";
    }
    return "UNKNOWN INTAKE MESSAGE";
}

struct Stopped : IntakeSys::State {
    void entry(IntakeSys &sys) override {
        sys.intake_lower.stop(vex::brakeType::brake);
        sys.intake_upper.stop(vex::brakeType::brake);
    }
    void exit(IntakeSys &sys) override {
        // cuz joe doesn't like his intake on brake mode
        sys.intake_lower.setBrake(vex::brakeType::coast);
        sys.intake_upper.setBrake(vex::brakeType::coast);
    }
    IntakeState id() const override { return IntakeState::Stopped; }
    State *respond(IntakeSys &sys, IntakeMessage m) override;
};

struct Dropping : IntakeSys::State {
    void entry(IntakeSys &sys) override {
        drop_timer.reset();
        sys.intake_upper.spin(vex::reverse, 12.0, vex::volt);
    }
    IntakeSys::MaybeMessage work(IntakeSys &sys) override {
        if (drop_timer.value() > intake_drop_seconds) {
            return IntakeMessage::Dropped;
        }
        return {};
    }
    void exit(IntakeSys &sys) override {
        sys.intake_upper.stop(vex::brakeType::coast);
    }
    IntakeState id() const override { return IntakeState::Dropping; }
    State *respond(IntakeSys &sys, IntakeMessage m) override;

  private:
    vex::timer drop_timer;
};
struct Intaking : IntakeSys::State {
    void entry(IntakeSys &sys) override {
        sys.intake_upper.spin(vex::fwd, intake_upper_volt, vex::volt);
        sys.intake_lower.spin(vex::fwd, intake_lower_volt, vex::volt);
    }
    void exit(IntakeSys &sys) override {
        sys.intake_upper.stop(vex::brakeType::coast);
        sys.intake_lower.stop(vex::brakeType::coast);
    }

    IntakeState id() const override { return IntakeState::Intaking; }
    State *respond(IntakeSys &sys, IntakeMessage m) override;
};
struct IntakingHold : IntakeSys::State {
    void entry(IntakeSys &sys) override {
        sys.intake_upper.spin(vex::fwd, intake_upper_volt_hold, vex::volt);
        sys.intake_lower.spin(vex::fwd, intake_lower_volt_hold, vex::volt);
    }
    IntakeSys::MaybeMessage work(IntakeSys &sys) override {
        if (sys.ball_in_intake()) {
            return IntakeMessage::StopIntake;
        }
        return {};
    }
    void exit(IntakeSys &sys) override {
        sys.intake_upper.stop(vex::brakeType::coast);
        sys.intake_lower.stop(vex::brakeType::coast);
    }

    IntakeState id() const override { return IntakeState::IntakingHold; }
    State *respond(IntakeSys &sys, IntakeMessage m) override;
};
struct Outtaking : IntakeSys::State {
    void entry(IntakeSys &sys) override {
        sys.intake_upper.spin(vex::reverse, intake_upper_outer_volt, vex::volt);
        sys.intake_lower.spin(vex::reverse, intake_lower_outer_volt, vex::volt);
    }
    void exit(IntakeSys &sys) override {
        sys.intake_upper.stop(vex::brakeType::coast);
        sys.intake_lower.stop(vex::brakeType::coast);
    }
    IntakeState id() const override { return IntakeState::Outtaking; }
    State *respond(IntakeSys &sys, IntakeMessage m) override;
};

IntakeSys::State *Dropping::respond(IntakeSys &sys, IntakeMessage m) {
    if (m == IntakeMessage::Dropped) {
        return new Stopped();
    }
    return this;
}

IntakeSys::State *Intaking::respond(IntakeSys &sys, IntakeMessage m) {
    if (m == IntakeMessage::StopIntake) {
        return new Stopped();
    } else if (m == IntakeMessage::IntakeHold) {
        return new IntakingHold();
    } else if (m == IntakeMessage::Outtake) {
        return new Outtaking;
    }
    return this;
}

IntakeSys::State *IntakingHold::respond(IntakeSys &sys, IntakeMessage m) {
    if (m == IntakeMessage::StopIntake) {
        return new Stopped();
    } else if (m == IntakeMessage::Intake) {
        return new Intaking();
    } else if (m == IntakeMessage::Outtake) {
        return new Outtaking;
    }
    return this;
}

IntakeSys::State *Stopped::respond(IntakeSys &sys, IntakeMessage m) {
    if (m == IntakeMessage::Intake) {
        return new Intaking();
    } else if (m == IntakeMessage::Outtake) {
        return new Outtaking();
    } else if (m == IntakeMessage::IntakeHold) {
        return new IntakingHold();
    }
    return this;
}

IntakeSys::State *Outtaking::respond(IntakeSys &sys, IntakeMessage m) {
    if (m == IntakeMessage::StopIntake) {
        return new Stopped();
    } else if (m == IntakeMessage::IntakeHold) {
        return new IntakingHold();
    } else if (m == IntakeMessage::Intake) {
        return new Intaking();
    }
    return this;
}

IntakeSys::IntakeSys(vex::distance &intake_watcher, vex::motor &intake_lower,
                     vex::motor &intake_upper, std::function<bool()> can_intake)
    : StateMachine(new Dropping()), intake_watcher(intake_watcher),
      intake_lower(intake_lower), intake_upper(intake_upper),
      can_intake(can_intake) {}

CataOnlySys::CataOnlySys(vex::pot &cata_pot, vex::optical &cata_watcher,
                         vex::motor_group &cata_motor, PIDFF &cata_pid)
    : StateMachine(new CataOff()), pot(cata_pot), cata_watcher(cata_watcher),
      mot(cata_motor), pid(cata_pid) {}

CataSys::CataSys(vex::distance &intake_watcher, vex::pot &cata_pot,
                 vex::optical &cata_watcher, vex::motor_group &cata_motor,
                 vex::motor &intake_upper, vex::motor &intake_lower,
                 PIDFF &cata_feedback)
    : intake_watcher(intake_watcher), cata_pot(cata_pot),
      cata_watcher(cata_watcher), cata_motor(cata_motor),
      intake_upper(intake_upper), intake_lower(intake_lower),
      cata_sys(cata_pot, cata_watcher, cata_motor, cata_feedback),
      intake_sys(intake_watcher, intake_lower, intake_upper,
                 [&]() { return cata_sys.intaking_allowed(); }) {}

void CataSys::send_command(Command next_cmd) {
    switch (next_cmd) {
    case CataSys::Command::StartFiring:
        cata_sys.send_message(CataOnlyMessage::Fire);
        break;
    case CataSys::Command::IntakeIn:
        if (cata_sys.current_state() == CataOnlyState::CataOff) {
            intake_sys.send_message(IntakeMessage::IntakeHold);
        } else if (cata_sys.intaking_allowed()) {
            intake_sys.send_message(IntakeMessage::Intake);
        }
        break;
    case CataSys::Command::IntakeOut:
        intake_sys.send_message(IntakeMessage::Outtake);
        break;
    case CataSys::Command::IntakeHold:
        if (cata_sys.intaking_allowed()) {
            intake_sys.send_message(IntakeMessage::IntakeHold);
        }
        break;
    case CataSys::Command::StopIntake:
        intake_sys.send_message(IntakeMessage::StopIntake);
        break;
    case CataSys::Command::StartDropping:
        intake_sys.send_message(IntakeMessage::Drop);
        cata_sys.send_message(CataOnlyMessage::StartDrop);
        break;
    case CataSys::Command::ToggleCata:
        if (cata_sys.current_state() == CataOnlyState::CataOff) {
            cata_sys.send_message(CataOnlyMessage::EnableCata);
        } else {
            cata_sys.send_message(CataOnlyMessage::DisableCata);
        }
        break;
    default:
        break;
    }
}

bool CataSys::can_fire() const {
    return cata_sys.current_state() == CataOnlyState::ReadyToFire;
}

class CataSysPage : public screen::Page {
  public:
    CataSysPage(const CataSys &cs)
        : gd(30, 130.0, 270.0, {vex::green, vex::red}, 2), cs(cs) {}
    void update(bool, int, int) override {}

    void draw(vex::brain::lcd &scr, bool, unsigned int) override {

        // Collect all the data
        CataOnlyState cata_state = cs.cata_sys.current_state();
        std::string cata_str = to_string(cata_state);

        IntakeState intake_state = cs.intake_sys.current_state();
        std::string intake_str = to_string(intake_state);

        gd.add_samples(
            {cs.cata_sys.pid.get_sensor_val(), cs.cata_sys.pid.get_target()});
        const bool ball_in_intake =
            cs.intake_watcher.objectDistance(distanceUnits::mm) <
            intake_sensor_dist_mm;

        const bool ball_in_cata = cs.cata_watcher.isNearObject();

        // Show it all
        scr.printAt(40, 20, true, "Cata: %s", cata_str.c_str());
        scr.printAt(40, 60, true, "pot: %.2f", cs.cata_pot.angle(vex::degrees));
        scr.printAt(40, 100, true, "Cata: %s", intake_str.c_str());
        scr.printAt(40, 120, true, "Cata Temp: %.0fC",
                    cs.cata_motor.temperature(vex::temperatureUnits::celsius));

        scr.printAt(40, 140, true, "Ball in Cata: %s",
                    ball_in_cata ? "yes" : "no");

        scr.printAt(40, 160, true, "Ball in Intake: %s",
                    ball_in_intake ? "yes" : "no");

        gd.draw(scr, 240, 0, 200, 200);
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

AutoCommand *CataSys::Unintake() {
    return new FunctionCommand([&]() {
        send_command(CataSys::Command::IntakeOut);
        return true;
    });
}
