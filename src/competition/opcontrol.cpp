#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{

    // ================ INIT ================
    const static double target_pos = 90.0;
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
        // printf("x: %f, y: %f, z: %f\n", 
        // imu.acceleration(xaxis), imu.acceleration(yaxis), imu.acceleration(zaxis));

        vexDelay(10);
    }

    // ================ PERIODIC ================
}