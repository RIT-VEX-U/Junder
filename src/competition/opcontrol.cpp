#include "competition/opcontrol.h"
#include "automation.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"
#include <atomic>

#define Tank

TankDrive::BrakeType brake_type = TankDrive::BrakeType::None;
auto toggle_brake_mode = []() {
    if (brake_type == TankDrive::BrakeType::None) {
        brake_type = TankDrive::BrakeType::Smart;
    } else {
        brake_type = TankDrive::BrakeType::None;
    }
};

/**
 * Main entrypoint for the driver control period
 */
void opcontrol() {
    // con.ButtonRight.pressed([]() { screen::next_page(); });
    // con.ButtonLeft.pressed([]() { screen::prev_page(); });
    //
    // autonomous();
    // return;
    cata_sys.send_command(CataSys::Command::StartDropping);

    while (imu.isCalibrating()) {
        vexDelay(20);
    }

    static bool enable_matchload = false;

#ifdef COMP_BOT
    static std::atomic<bool> disable_drive(false);

    con.ButtonRight.pressed([]() {
        // Turn Right
        disable_drive = true;
        right_motors.spin(directionType::rev, 5, volt);
        left_motors.spin(directionType::fwd, 3, volt);
        vexDelay(150);
        right_motors.stop(brakeType::hold);
        left_motors.stop(brakeType::hold);
        vexDelay(150);
        right_motors.stop(brakeType::coast);
        left_motors.stop(brakeType::coast);
        disable_drive = false;
    });

    con.ButtonLeft.pressed([]() {
        // Turn Left
        disable_drive = true;
        right_motors.spin(directionType::fwd, 3, volt);
        left_motors.spin(directionType::rev, 5, volt);
        vexDelay(150);
        right_motors.stop(brakeType::hold);
        left_motors.stop(brakeType::hold);
        vexDelay(150);
        right_motors.stop(brakeType::coast);
        left_motors.stop(brakeType::coast);
        disable_drive = false;
    });
    con.ButtonDown.pressed(
        []() { climb_solenoid.set(!climb_solenoid.value()); });
    con.ButtonUp.pressed([]() {
        CommandController cc{Climb()};
        cc.run();
    });

    con.ButtonA.pressed([]() { enable_matchload = !enable_matchload; });

    con.ButtonL1.pressed(
        []() { cata_sys.send_command(CataSys::Command::StartFiring); });
    con.ButtonL1.released(
        []() { cata_sys.send_command(CataSys::Command::StopFiring); });
    con.ButtonR1.pressed(
        []() { cata_sys.send_command(CataSys::Command::IntakeIn); });
    con.ButtonR2.pressed(
        []() { cata_sys.send_command(CataSys::Command::IntakeOut); });

    con.ButtonY.pressed(
        []() { cata_sys.send_command(CataSys::Command::IntakeHold); });

    con.ButtonL2.pressed([]() {
        left_wing.set(!left_wing.value());
        right_wing.set(!right_wing.value());
    });

#endif
    con.ButtonB.pressed([]() { toggle_brake_mode(); });
    // ================ INIT ================
    while (true) {
#ifdef COMP_BOT
        if (!con.ButtonR1.pressing() && !con.ButtonR2.pressing() &&
            !con.ButtonY.pressing()) {
            cata_sys.send_command(CataSys::Command::StopIntake);
        }
#endif
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        if (!disable_drive) {
            drive_sys.drive_tank(l, r, 1, brake_type);
        }

#else

        double f = con.Axis3.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        drive_sys.drive_arcade(f, s, 1, brake_type);
#endif

        // matchload_1(enable_matchload); // Toggle
        matchload_1([]() { return con.ButtonA.pressing(); }); // Hold
        // Controls
        // Intake

        vexDelay(10);
    }

    // ================ PERIODIC ================
}
