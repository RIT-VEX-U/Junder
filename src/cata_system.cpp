#include "cata_system.h"
#include <string>

CataSys::CataSys(vex::distance &intake_watcher, vex::pot &cata_pot,
                 vex::optical &cata_watcher, vex::motor_group &cata_motor,
                 vex::motor &intake_upper, vex::motor &intake_lower,
                 PIDFF &cata_feedback, DropMode drop)
    : intake_watcher(intake_watcher), cata_pot(cata_pot),
      cata_watcher(cata_watcher), cata_motor(cata_motor),
      intake_upper(intake_upper), intake_lower(intake_lower),
      cata_sys(cata_pot, cata_watcher, cata_motor, cata_feedback, drop),
      intake_sys(
          intake_watcher, intake_lower, intake_upper,
          [&]() { return cata_sys.intaking_allowed(); }, drop) {}

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
        if (cata_sys.current_state() == CataOnlyState::CataOff) {
            intake_sys.send_message(IntakeMessage::IntakeHold);
        } else if (cata_sys.intaking_allowed()) {
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
bool CataSys::still_dropping() {
    bool still_dropping =
        cata_sys.current_state() == CataOnlyState::WaitingForDrop ||
        intake_sys.current_state() == IntakeState::Dropping;
    return !still_dropping;
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
            {cs.cata_pot.angle(vex::deg), cs.cata_sys.pid.get_target()});
        const bool ball_in_intake =
            cs.intake_watcher.objectDistance(distanceUnits::mm) <
            intake_sensor_dist_mm;

        const bool ball_in_cata = cs.cata_watcher.isNearObject();

        // Show it all
        scr.printAt(40, 20, true, "Cata: %s", cata_str.c_str());
        scr.printAt(40, 60, true, "pot: %.2f", cs.cata_pot.angle(vex::degrees));
        scr.printAt(40, 100, true, "Intake: %s", intake_str.c_str());
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
