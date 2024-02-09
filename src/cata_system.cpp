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
const double fire_time = 0.1;
const double fire_voltage = 12.0;

bool intake_can_be_enabled(double cata_pos) {
    return (cata_pos == 0.0) || (cata_pos > inake_enable_lower_threshold &&
                                 cata_pos < intake_enable_upper_threshold);
}
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
            printf("no adc\n");
            fflush(stdout);
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
    CataOnlyState id() const override {

        printf("ready to fire id\n");
        return CataOnlyState::ReadyToFire;
    }
};
CataOnlySys::State *Reloading::respond(CataOnlyMessage m) {
    if (m == CataOnlyMessage::DoneReloading) {
        auto t = new ReadyToFire();

        return t;
    }
    printf("Reloading chilling\n");
    fflush(stdout);
    // Ignore other messages
    return this;
}
CataOnlySys::State *ReadyToFire::respond(CataOnlyMessage m) {
    if (m == CataOnlyMessage::Fire) {
        printf("Now Firing\n");
        fflush(stdout);
        return new Firing();
    } else if (m == CataOnlyMessage::Slipped) {
        printf("Now reloading\n");
        fflush(stdout);
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
      cata_sys(cata_pot, cata_watcher, cata_motor, cata_feedback) {
    printf("CataSys\n");
    fflush(stdout);
}

void CataSys::send_command(Command next_cmd) {
    switch (next_cmd) {
    case CataSys::Command::StartFiring:
        // firing_requested = true;
        cata_sys.SendMessage(CataOnlyMessage::Fire);
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
        CataOnlyState cata_state = cs.cata_sys.current_state();
        std::string cata_str = to_string(cata_state);

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

        gd.add_samples(
            {cs.cata_sys.pid.get_sensor_val(), cs.cata_sys.pid.get_target()});
        //
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

        scr.printAt(40, 20, true, "Cata: %s", cata_str.c_str());
        scr.printAt(40, 60, true, "pot: %.2f", cs.cata_pot.angle(vex::degrees));
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
