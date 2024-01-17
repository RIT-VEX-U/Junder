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
    con.ButtonRight.pressed([]() { screen::next_page(); });
    con.ButtonLeft.pressed([]() { screen::prev_page(); });

    con.ButtonY.pressed([]() {
        auto pose = odom.get_position();

        printf("(%.2f, %.2f) - %.2fdeg\n", pose.x, pose.y, pose.rot);
    });

    autonomous();
    // return;
    cata_sys.send_command(CataSys::Command::StartDropping);

    while (imu.isCalibrating()) {
        vexDelay(20);
    }

    con.ButtonRight.pressed([]() { screen::next_page(); });
    con.ButtonLeft.pressed([]() { screen::prev_page(); });
    con.ButtonDown.pressed(
        []() { climb_solenoid.set(!climb_solenoid.value()); });

    con.ButtonY.pressed([]() {
        auto pose = odom.get_position();

        printf("(%.2f, %.2f) - %.2fdeg\n", pose.x, pose.y, pose.rot);
    });
    // CommandController cc {
    // drive_sys.DriveForwardCmd(6.0, vex::fwd, 0.2),
    // };
    // cc.run();
    // skills();

    static bool enable_matchload = false;

    // Controls:
    // Cata: Hold L1 (Not on rising edge)
    // -- Don't shoot until there's a ball
    // -- Preload
    // Intake:
    // -- R1 IN
    // -- R2 OUT
    // -- B - 2 intake pistons

    // SUBJECT TO CHANGE!
    // Wings: DOWN

#ifdef COMP_BOT
    con.ButtonA.pressed([]() { enable_matchload = !enable_matchload; });

    con.ButtonL1.pressed(
        []() { cata_sys.send_command(CataSys::Command::StartFiring); });
    con.ButtonL1.released(
        []() { cata_sys.send_command(CataSys::Command::StopFiring); });
    con.ButtonR1.pressed(
        []() { cata_sys.send_command(CataSys::Command::IntakeIn); });
    con.ButtonR2.pressed(
        []() { cata_sys.send_command(CataSys::Command::IntakeOut); });

    con.ButtonUp.pressed(
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
            !con.ButtonL2.pressing() && !con.ButtonUp.pressing()) {
            cata_sys.send_command(CataSys::Command::StopIntake);
        }
#endif
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        drive_sys.drive_tank(l, r, 1, brake_type);

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