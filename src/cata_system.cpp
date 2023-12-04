#include "cata_system.h"
#include "robot-config.h"

// const double inake_enable_lower_threshold = 70.0;
// const double intake_enable_upper_threshold = 120;
const double inake_enable_lower_threshold = 150;
const double intake_enable_upper_threshold = 200;

const double intake_upper_volt = 12;
const double intake_upper_volt_hold = 6;
const double intake_lower_volt = 9.0;
const double intake_sensor_dist_mm = 150;

const double cata_target_charge = 178;
const double cata_target_intake = 178; // LOWER IS CLOSER TO SLIPPPING

PID::pid_config_t pc = {.p = 1,
                        // .i = 2,
                        .deadband = 2,
                        .on_target_time = 0.3};

FeedForward::ff_config_t ffc = {.kG = -2};

PIDFF cata_pid(pc, ffc);

bool intake_can_be_enabled(double cata_pos) {
    return (cata_pos == 0.0) || (cata_pos > inake_enable_lower_threshold &&
                                 cata_pos < intake_enable_upper_threshold);
}

int thread_func(void *void_cata) {
    CataSys &cata = *(CataSys *)void_cata;
    cata.state = CataSys::CataState::CHARGING;

    cata_pid.set_limits(-12.0, 0);

    vex::timer intake_tmr;

    while(DONT_RUN_CATA_YOU_FOOL){
        vexDelay(20);
    }
    // cata_motors.stop(brakeType::hold);
    // for (int i = 0; i < 10; i++) {
    //     cata.intake_lower.spin(vex::reverse, 12.0, vex::volt);
    //     vexDelay(20);
    // }

    while (true) {
        // read sensors
        const double cata_pos = cata.cata_pot.angle(vex::degrees);

        if (cata.intake_watcher.objectDistance(distanceUnits::mm) <
            intake_sensor_dist_mm) {
            intake_tmr.reset();
        }

        const bool ball_in_intake =
            cata.intake_watcher.objectDistance(distanceUnits::mm) <
                intake_sensor_dist_mm ||
            intake_tmr.time(timeUnits::msec) < 500;

        const bool ball_in_cata = cata.cata_watcher.isNearObject();

        // SYNCHRONIZE (READ)
        cata.control_mut.lock();
        CataSys::CataState cur_state = cata.state;
        bool firing_requested = cata.firing_requested;
        bool intaking_requested = cata.intaking_requested;
        bool matchload_requested = cata.matchload_requested;
        CataSys::IntakeType intake_type = cata.intake_type;
        cata.control_mut.unlock();

        bool intake_cata_enabled = false;
        double cata_pid_out = 0;

        // Main Intake State Machine
        switch (cur_state) {
        case CataSys::CataState::FIRING:
            cata.intake_upper.stop();
            cata.intake_lower.stop();
            break;
        case CataSys::CataState::CHARGING:
        case CataSys::CataState::READY:
            // Charging & Ready: Base off of cata position
            // ==== INTAKE ====
            // Run based on requested action
            // Make sure the cata is in a good position

            if (intaking_requested && intake_can_be_enabled(cata_pos) &&
                intake_type == CataSys::IntakeType::In && !ball_in_cata) {
                // Intake triball
                cata.intake_upper.spin(vex::fwd, intake_upper_volt, vex::volt);
                cata.intake_lower.spin(vex::fwd, intake_lower_volt, vex::volt);
            } else if (intaking_requested && intake_can_be_enabled(cata_pos) &&
                       intake_type == CataSys::IntakeType::Hold &&
                       !ball_in_cata) {
                // Intake until ball is sensed in intake, then stop
                // timer to make sure it doesn't go straight through
                if (ball_in_intake) {
                    cata.intake_upper.stop(brakeType::hold);
                    cata.intake_lower.stop(brakeType::hold);
                } else {
                    cata.intake_upper.spin(vex::fwd, intake_upper_volt_hold,
                                           vex::volt);
                    cata.intake_lower.spin(vex::fwd, intake_lower_volt,
                                           vex::volt);
                }
            } else if (intaking_requested &&
                       intake_type == CataSys::IntakeType::Out) {
                cata.intake_upper.spin(vex::fwd, -intake_upper_volt, vex::volt);
                cata.intake_lower.spin(vex::fwd, -intake_lower_volt, vex::volt);
            } else {
                cata.intake_upper.stop();
                cata.intake_lower.stop();
            }

            // Reset requests from AutoCommands
            if ((intake_type == CataSys::IntakeType::Hold && ball_in_intake) ||
                (intake_type == CataSys::IntakeType::In && ball_in_cata)) {
                // intaking_requested = false;
            }
        }

        // Main catapult state machine
        switch (cur_state) {
        case CataSys::CataState::CHARGING:

            // ==== CATAPULT ===
            // Run via PID
            cata_pid.set_target(cata_target_charge);
            cata_pid_out = cata_pid.update(cata_pos);
            cata.cata_motor.spin(vex::fwd, cata_pid_out, vex::volt);

            // ==== EXIT STATE ====
            // Ratchet engaged, we are READY
            if (cata_pid.is_on_target() ||
                cata_pos < cata_target_charge - pc.deadband) {
                cur_state = CataSys::CataState::READY;
            }
            break;
        case CataSys::CataState::READY:

            // ==== CATAPULT ====
            // Disable, rely on ratchet

            if (intake_cata_enabled == false) {
                // cata.cata_motor.stop(brakeType::coast);

                // REMOVE IF RATCHETING
                cata_pid.set_target(cata_target_charge);
                cata_pid_out = cata_pid.update(cata_pos);
                cata.cata_motor.spin(vex::fwd, cata_pid_out, vex::volt);
            }

            // When firing is requested, FIRE!
            if (firing_requested && cata.can_fire()) {
                cur_state = CataSys::CataState::FIRING;
            }

            // Check if position is too high, go back to charging.
            if (cata_pos > cata_target_charge + 30) {
                cur_state = CataSys::CataState::CHARGING;
            }
            break;
        case CataSys::CataState::FIRING:

            // ==== CATAPULT ====
            cata.cata_motor.spinFor(directionType::rev, 500, timeUnits::msec,
                                    100.0, velocityUnits::pct);
            cata_pid.reset(); // reset integral

            if (cata.cata_motor.isDone()) {
                cur_state = CataSys::CataState::CHARGING;
                firing_requested = false;
            }
            break;
        }

        if (cata.cata_watcher.isNearObject() &&
            cata.intake_watcher.objectDistance(distanceUnits::mm) < 150) {
            printf(
                "DQed! ball in intake and catapault (or a sensor got funny)\n");
            con.rumble("-");
        }

        // SYNCHRONIZE
        cata.control_mut.lock();
        cata.state = cur_state;
        cata.firing_requested = firing_requested;
        cata.intaking_requested = intaking_requested;
        cata.control_mut.unlock();

        vexDelay(5);
    }
    return 0;
}

CataSys::CataSys(vex::distance &intake_watcher, vex::pot &cata_pot,
                 vex::optical &cata_watcher, vex::motor_group &cata_motor,
                 vex::motor &intake_upper, vex::motor &intake_lower)
    : intake_watcher(intake_watcher), cata_pot(cata_pot),
      cata_watcher(cata_watcher), cata_motor(cata_motor),
      intake_upper(intake_upper), intake_lower(intake_lower),
      firing_requested(false), intaking_requested(false),
      intake_type(CataSys::IntakeType::In) {
    runner = vex::task(thread_func, (void *)this);
}

void CataSys::send_command(Command next_cmd) {
    control_mut.lock();
    switch (next_cmd) {
    case CataSys::Command::StartFiring:
        firing_requested = true;
        break;
    case CataSys::Command::StopFiring:
        firing_requested = false;
        break;
    case CataSys::Command::IntakeIn:
        intaking_requested = true;
        intake_type = CataSys::IntakeType::In;
        break;
    case CataSys::Command::IntakeOut:
        intaking_requested = true;
        intake_type = CataSys::IntakeType::Out;
        break;
    case CataSys::Command::IntakeHold: // not handled
        intaking_requested = true;
        intake_type = CataSys::IntakeType::Hold;
        break;
    case CataSys::Command::StopIntake:
        intaking_requested = false;
        break;
    case CataSys::Command::StartMatchLoad:
        matchload_requested = true;
        break;
    case CataSys::Command::StopMatchLoad:
        matchload_requested = false;
        break;
    default:
        break;
    }
    control_mut.unlock();
}

CataSys::CataState CataSys::get_state() const {
    control_mut.lock();
    auto s = state;
    control_mut.unlock();
    return s;
}

bool CataSys::can_fire() const {
    return cata_watcher.isNearObject() && get_state() == CataState::READY;
}

class CataSysPage : public screen::Page {
  public:
    CataSysPage(const CataSys &cs)
        : gd(30, 130.0, 270.0, {vex::green, vex::red}, 2), cs(cs) {}
    void update(bool, int, int) override {}

    void draw(vex::brain::lcd &scr, bool, unsigned int) override {
        CataSys::CataState state = cs.get_state();
        const char *state_str = "UNKNOWN STATE";
        switch (state) {
        case CataSys::CataState::CHARGING:
            state_str = "CHARGING";
            break;
        case CataSys::CataState::FIRING:
            state_str = "FIRING";
            break;
        case CataSys::CataState::READY:
            state_str = "READY";
            break;

        default:
            break;
        }

        gd.add_samples({cata_pid.get_sensor_val(), cata_pid.get_target()});

        const bool ball_in_intake =
            cs.intake_watcher.objectDistance(distanceUnits::mm) <
            intake_sensor_dist_mm;

        const bool ball_in_cata = cs.cata_watcher.isNearObject();

        const char *intake_dir = "";
        if (cs.intaking_requested) {
            switch (cs.intake_type) {
            case CataSys::IntakeType::Hold:
                intake_dir = "Hold";
                break;
            case CataSys::IntakeType::In:
                intake_dir = "In";
                break;
            case CataSys::IntakeType::Out:
                intake_dir = "Out";
                break;
            }
        }

        scr.printAt(40, 20, true, "Cata: %s", state_str);
        scr.printAt(40, 60, true, "MatchLoad Requested: %s",
                    cs.matchload_requested ? "yes" : "no");
        scr.printAt(40, 80, true, "Intake Requested: %s%s",
                    cs.intaking_requested ? "yes" : "no", intake_dir);
        scr.printAt(40, 100, true, "Fire Requested: %s",
                    cs.firing_requested ? "yes" : "no");

        scr.printAt(40, 140, true, "Ball in Cata: %s",
                    ball_in_cata ? "yes" : "no");

        scr.printAt(40, 160, true, "Ball in Intake: %s",
                    ball_in_intake ? "yes" : "no");
        scr.printAt(40, 180, true, "Can Fire: %s",
                    cs.can_fire() ? "yes" : "no");

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

AutoCommand *CataSys::WaitForIntake() {
    return new FunctionCommand([&]() {
        return intake_watcher.objectDistance(distanceUnits::mm) < 150;
    });
}

AutoCommand *CataSys::StopFiring() {
    return new FunctionCommand([]() {
        cata_sys.send_command(CataSys::Command::StopFiring);
        return true;
    });
}