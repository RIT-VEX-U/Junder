#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

#define Tank

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    while (imu.isCalibrating()) // || gps_sensor.isCalibrating())
    {
        vexDelay(20);
    }

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

    // ================ INIT ================
    while (true)
    {
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