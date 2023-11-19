#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

#define Tank


/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    // while (imu.isCalibrating())// || gps_sensor.isCalibrating())
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

    con.ButtonA.pressed([](){cata_motors.spin(vex::fwd, 12.0, vex::volt);});
    con.ButtonA.released([](){cata_motors.stop(vex::brakeType::hold);});
    // ================ INIT ================
    while (true)
    {
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        drive_sys.drive_tank(l, r, 1, TankDrive::BrakeType::None);

#else

        double f = con.Axis3.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        drive_sys.drive_arcade(f, s);
#endif

        // Controls
        // Intake
        if(con.ButtonR1.pressing())
        {
            intake_combine.spin(directionType::fwd, 12, volt);
            intake_roller.spin(directionType::fwd, 12, volt);
        } else if(con.ButtonR2.pressing())
        {
            intake_combine.spin(directionType::rev, 12, volt);
            intake_roller.spin(directionType::rev, 12, volt);
        } else
        {
            intake_combine.stop();
            intake_roller.stop();
        }
        

        vexDelay(10);
    }

    // ================ PERIODIC ================
}