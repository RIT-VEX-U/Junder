#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"
#include "automation.h"
#include <atomic>

// #define Tank

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    // vexDelay(1000);

    while (imu.isCalibrating()) // || gps_sensor.isCalibrating())
    {
        vexDelay(20);
    }

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
    con.ButtonL1.pressed(   [](){ cata_sys.send_command(CataSys::Command::StartFiring); });
    con.ButtonL1.released(  [](){ cata_sys.send_command(CataSys::Command::StopFiring); });
    con.ButtonR1.pressed(   [](){ cata_sys.send_command(CataSys::Command::IntakeIn); });
    con.ButtonR2.pressed(   [](){ cata_sys.send_command(CataSys::Command::IntakeOut); });
    con.ButtonL2.pressed(   [](){ cata_sys.send_command(CataSys::Command::IntakeHold); });
    con.ButtonDown.pressed( [](){ left_wing.set(!left_wing.value()); });
    con.ButtonB.pressed(    [](){ right_wing.set(!right_wing.value()); });
    con.ButtonA.pressed(    [](){ enable_matchload = !enable_matchload; });

#endif
    // ================ INIT ================
    while (true)
    {
        if (!con.ButtonR1.pressing() && !con.ButtonR2.pressing() && !con.ButtonL2.pressing())
        {
            cata_sys.send_command(CataSys::Command::StopIntake);
        }
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::Smart);

#else

        double f = con.Axis3.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        drive_sys.drive_arcade(f, s);
#endif

        // matchload_1(enable_matchload); // Toggle
        matchload_1([](){ return con.ButtonA.pressing();}); // Hold
        // Controls
        // Intake

        vexDelay(10);
    }

    // ================ PERIODIC ================
}