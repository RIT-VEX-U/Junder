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
const double intake_lower_volt_hold = 6;
const double intake_lower_volt = 9.0;
const double intake_sensor_dist_mm = 150;

const double cata_target_charge = 179;
const double cata_target_intake = 179; // LOWER IS CLOSER TO SLIPPPING

const double intake_drop_seconds = 0.5;
const double fire_time = 0.1;
const double fire_voltage = 12.0;

bool intake_can_be_enabled(double cata_pos) {
    return (cata_pos == 0.0) || (cata_pos > inake_enable_lower_threshold &&
                                 cata_pos < intake_enable_upper_threshold);
}

// Cata
// ==============================================================================================================================
std::string to_string(CataOnlyState s) {
    switch (s) {
    // case CataOnlyState::Starting:
    // return "Starting";
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
    case CataOnlyMessage::Slipped:
        return "Slipped";
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

        sys.pid.update(sys.pot.angle(vex::deg));
        sys.pid.set_target(cata_target_charge);
    }

    CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
        // work on motor
        double cata_deg = sys.pot.angle(vex::deg);
        if (cata_deg == 0.0) {
            // adc hasnt warmed up yet
            return {};
        }

        sys.pid.update(cata_deg);
        printf("volt: %.2f - %.1f\n", sys.pid.get(), cata_deg);
        sys.mot.spin(vex::fwd, sys.pid.get(), vex::volt);
        // are we there yettt
        if (sys.pid.is_on_target()) {
            printf("Reloading done\n");
            fflush(stdout);
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
        sys.mot.spin(vex::reverse, fire_voltage, vex::volt);
    }
    CataOnlySys::MaybeMessage work(CataOnlySys &sys) override {
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

        // If we slipped, send message to go back to reload
        if (!intake_can_be_enabled(cata_deg)) {
            printf("Slipped\n");
            fflush(stdout);
            return CataOnlyMessage::Slipped;
        }

        // hold here until message comes from outside
        return {};
    }
    State *respond(CataOnlyMessage m) override;
    CataOnlyState id() const override { return CataOnlyState::ReadyToFire; }
};
CataOnlySys::State *Reloading::respond(CataOnlyMessage m) {
    if (m == CataOnlyMessage::DoneReloading) {
        auto t = new ReadyToFire();

        return t;
    }
    // Ignore other messages
    return this;
}
CataOnlySys::State *ReadyToFire::respond(CataOnlyMessage m) {
    if (m == CataOnlyMessage::Fire) {
        return new Firing();
    } else if (m == CataOnlyMessage::Slipped) {
        return new Reloading();
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
// INTAKE
// ==============================================================================================================================
bool IntakeSys::can_intake() { return true; }
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
    State *respond(IntakeMessage m) override;
};

struct Dropping : IntakeSys::State {
    void entry(IntakeSys &sys) override {
        tmr.reset();
        sys.intake_upper.spin(vex::reverse, 12.0, vex::volt);
    }
    IntakeSys::MaybeMessage work(IntakeSys &sys) override {
        if (tmr.value() > intake_drop_seconds) {
            return IntakeMessage::Dropped;
        }
        return {};
    }
    void exit(IntakeSys &sys) override {
        sys.intake_upper.stop(vex::brakeType::coast);
    }
    IntakeState id() const override { return IntakeState::Dropping; }
    State *respond(IntakeMessage m) override;

  private:
    vex::timer tmr;
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
    State *respond(IntakeMessage m) override;
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
    State *respond(IntakeMessage m) override;
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
    State *respond(IntakeMessage m) override;
};

IntakeSys::State *Dropping::respond(IntakeMessage m) {
    if (m == IntakeMessage::Dropped) {
        return new Stopped();
    }
    return this;
}

IntakeSys::State *Intaking::respond(IntakeMessage m) {
    if (m == IntakeMessage::StopIntake) {
        return new Stopped();
    } else if (m == IntakeMessage::IntakeHold) {
        return new IntakingHold();
    } else if (m == IntakeMessage::Outtake) {
        return new Outtaking;
    }
    return this;
}

IntakeSys::State *IntakingHold::respond(IntakeMessage m) {
    if (m == IntakeMessage::StopIntake) {
        return new Stopped();
    } else if (m == IntakeMessage::Intake) {
        return new Intaking();
    } else if (m == IntakeMessage::Outtake) {
        return new Outtaking;
    }
    return this;
}

IntakeSys::State *Stopped::respond(IntakeMessage m) {
    if (m == IntakeMessage::Intake) {
        return new Intaking();
    } else if (m == IntakeMessage::Outtake) {
        return new Outtaking();
    } else if (m == IntakeMessage::IntakeHold) {
        return new IntakingHold();
    }
    return this;
}

IntakeSys::State *Outtaking::respond(IntakeMessage m) {
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
                     vex::motor &intake_upper)
    : intake_watcher(intake_watcher), intake_lower(intake_lower),
      intake_upper(intake_upper), StateMachine(new Dropping()) {}

CataOnlySys::CataOnlySys(vex::pot &cata_pot, vex::optical &cata_watcher,
                         vex::motor_group &cata_motor, PIDFF &cata_pid)
    : StateMachine(new Reloading()), pot(cata_pot), cata_watcher(cata_watcher),
      mot(cata_motor), pid(cata_pid) {}

CataSys::CataSys(vex::distance &intake_watcher, vex::pot &cata_pot,
                 vex::optical &cata_watcher, vex::motor_group &cata_motor,
                 vex::motor &intake_upper, vex::motor &intake_lower,
                 PIDFF &cata_feedback)
    : intake_watcher(intake_watcher), cata_pot(cata_pot),
      cata_watcher(cata_watcher), cata_motor(cata_motor),
      intake_upper(intake_upper), intake_lower(intake_lower),
      cata_sys(cata_pot, cata_watcher, cata_motor, cata_feedback),
      intake_sys(intake_watcher, intake_lower, intake_upper) {}

void CataSys::send_command(Command next_cmd) {
    switch (next_cmd) {
    case CataSys::Command::StartFiring:
        cata_sys.SendMessage(CataOnlyMessage::Fire);
        break;
    case CataSys::Command::IntakeIn:
        intake_sys.SendMessage(IntakeMessage::Intake);
        break;
    case CataSys::Command::IntakeOut:
        intake_sys.SendMessage(IntakeMessage::Outtake);
        break;
    case CataSys::Command::IntakeHold:
        intake_sys.SendMessage(IntakeMessage::IntakeHold);
        break;
    case CataSys::Command::StopIntake:
        intake_sys.SendMessage(IntakeMessage::StopIntake);
        break;
    case CataSys::Command::StartDropping:
        intake_sys.SendMessage(IntakeMessage::Drop);
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

        scr.printAt(40, 20, true, "Cata: %s", cata_str.c_str());
        scr.printAt(40, 60, true, "pot: %.2f", cs.cata_pot.angle(vex::degrees));
        scr.printAt(40, 100, true, "Cata: %s", intake_str.c_str());

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
