#include "competition/opcontrol.h"
#include "automation.h"
#include "competition/autonomous.h"
#include "robot-config.h"
#include "vex.h"
#include <atomic>

bool Tank = false;

TankDrive::BrakeType brake_type = TankDrive::BrakeType::Smart;

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
    //
    cata_sys.send_command(CataSys::Command::StartDropping);

    while (imu.isCalibrating()) {
        vexDelay(20);
    }

    static bool enable_matchload = false;
    static std::atomic<bool> disable_drive(false);

#ifdef COMP_BOT

    con.ButtonDown.pressed(
        []() { climb_solenoid.set(!climb_solenoid.value()); });

    con.ButtonA.pressed([]() { enable_matchload = !enable_matchload; });

    con.ButtonL1.pressed(
        []() { cata_sys.send_command(CataSys::Command::StartFiring); });
    con.ButtonR1.pressed(
        []() { cata_sys.send_command(CataSys::Command::IntakeIn); });
    con.ButtonR2.pressed(
        []() { cata_sys.send_command(CataSys::Command::IntakeOut); });

    con.ButtonY.pressed([]() { Tank = !Tank; });
    con.ButtonUp.pressed(
        []() { cata_sys.send_command(CataSys::Command::ToggleCata); });
    con.ButtonL2.pressed([]() {
        left_wing.set(!left_wing.value());
        right_wing.set(!right_wing.value());
    });

#endif
    con.ButtonB.pressed([]() { toggle_brake_mode(); });
    // ================ INIT ================
    while (true) {
        if (!con.ButtonR1.pressing() && !con.ButtonR2.pressing()) {
            if (!cata_sys.still_dropping()) {
                cata_sys.send_command(CataSys::Command::StopIntake);
            }
        }

        if (Tank) {
            double l = con.Axis3.position() / 100.0;
            double r = con.Axis2.position() / 100.0;
            if (!disable_drive) {
                drive_sys.drive_tank(l, r, 1, brake_type);
            }

        } else {

            double f = con.Axis3.position() / 100.0;
            double s = con.Axis1.position() / 100.0;
            if (!disable_drive) {
                drive_sys.drive_arcade(f, s, 1, brake_type);
            }
        }
        static VisionTrackTriballCommand viscmd;

        if (con.ButtonX.pressing()) {
            disable_drive = true;
            bool override_watch = false;
#ifndef COMP_BOT
            override_watch = true;
#endif
            if (override_watch ||
                intake_watcher.objectDistance(distanceUnits::mm) > 100.0) {
                viscmd.run();
            } else {
                drive_sys.stop();
                cata_sys.send_command(CataSys::Command::StopIntake);
            }
        } else {
            disable_drive = false;
        }
        // Controls
        // Intake

        vexDelay(10);
    }

    // ================ PERIODIC ================
}