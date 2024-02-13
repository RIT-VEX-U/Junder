#include "cata/intake.h"
#include "vex.h"

// INTAKE
// ==============================================================================================================================

bool IntakeSys::ball_in_intake() {
    return intake_watcher.objectDistance(vex::distanceUnits::mm) <
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
    case IntakeState::IntakeWaitForDrop:
        return "IntakeWaitForDrop";
    default:
        return "UNKNOWN INTAKE STATE";
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

struct IntakeWaitForDrop : IntakeSys::State {
    IntakeState id() const override { return IntakeState::IntakeWaitForDrop; }
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

IntakeSys::State *IntakeWaitForDrop::respond(IntakeSys &sys, IntakeMessage m) {
    if (m == IntakeMessage::Drop) {
        return new Dropping();
    }
    return this;
}

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
                     vex::motor &intake_upper, std::function<bool()> can_intake,
                     DropMode drop)
    : StateMachine(drop == DropMode::Required
                       ? (IntakeSys::State *)(new IntakeWaitForDrop())
                       : (IntakeSys::State *)(new Stopped())),
      intake_watcher(intake_watcher), intake_lower(intake_lower),
      intake_upper(intake_upper), can_intake(can_intake) {}
