#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"
#include "automation.h"

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

    pose_t start_pose = {.x = 16, .y = 144 - 16, .rot = 135};

    CommandController cc{
        new RepeatUntil(
            {
                odom.SetPositionCmd(start_pose),
                cata_sys.IntakeFully(),
                drive_sys.DriveForwardCmd(4, directionType::rev),
                cata_sys.Fire(),
                drive_sys.DriveForwardCmd(4, directionType::fwd),

            },
            10),
        // drive_sys.DriveForwardCmd(36, vex::directionType::rev, 0.9),
        // drive_sys.TurnToHeadingCmd(-90),
        // drive_sys.DriveToPointCmd({.x = 27, .y = 18}, vex::fwd)
    };
    cc.add_cancel_func([]()
                       { return con.ButtonA.pressing(); });
    cc.run();
    // while(true){
    // vexDelay(1000);
    // }
    // return;

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
    con.ButtonL1.pressed([]()
                         { cata_sys.send_command(CataSys::Command::StartFiring); });
    con.ButtonL1.released([]()
                          { cata_sys.send_command(CataSys::Command::StopFiring); });
    con.ButtonR1.pressed([]()
                         { cata_sys.send_command(CataSys::Command::IntakeIn); });
    con.ButtonR2.pressed([]()
                         { cata_sys.send_command(CataSys::Command::IntakeOut); });
    con.ButtonL2.pressed([]()
                         { cata_sys.send_command(CataSys::Command::IntakeHold); });

    con.ButtonRight.pressed([]()
                            { cata_sys.send_command(CataSys::Command::StartMatchLoad); });
    con.ButtonLeft.pressed([]()
                           { cata_sys.send_command(CataSys::Command::StopMatchLoad); });

    con.ButtonDown.pressed([]()
                           { left_wing.set(!left_wing.value()); });
    con.ButtonB.pressed([]()
                        { right_wing.set(!right_wing.value()); });

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
        drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::Smart);
#endif

        // matchload_1(con.ButtonA.pressing());
        // Controls
        // Intake

        vexDelay(10);
    }

    // ================ PERIODIC ================
}