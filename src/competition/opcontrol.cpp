#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    imu.orient(xaxis, yaxis, zaxis, true);
    imu.calibrate();
    
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
        printf("x: %f, y: %f, z: %f\n", 
        imu.get_accel(xaxis).pos, imu.get_accel(yaxis).pos, imu.get_accel(zaxis).pos);

        vexDelay(10);
    }

    // ================ PERIODIC ================
}