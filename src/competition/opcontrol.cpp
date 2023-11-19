#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

#define Tank

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    // while (imu.isCalibrating()) // || gps_sensor.isCalibrating())
    // {
    //     vexDelay(20);
    // }

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
    
    con.ButtonL1.pressed([]()
                         {  cata_sys.send_command(CataSys::Command::StartFiring); });
    con.ButtonL1.released([]()
                          {  cata_sys.send_command(CataSys::Command::StopFiring); });
    con.ButtonR1.pressed([]()
                         {  cata_sys.send_command(CataSys::Command::IntakeIn); });
    con.ButtonR2.pressed([]()
                         {  cata_sys.send_command(CataSys::Command::IntakeOut); });
    // ================ INIT ================
    while (true)
    {
        if (!con.ButtonR1.pressing() && !con.ButtonR2.pressing())
        {
            cata_sys.send_command(CataSys::Command::StopIntake);
        }
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        drive_sys.drive_tank(l, r);
#else

        double f = con.Axis2.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        drive_sys.drive_arcade(f, s);
#endif

        // Controls
        // Intake

        vexDelay(10);
    }

    // ================ PERIODIC ================
}