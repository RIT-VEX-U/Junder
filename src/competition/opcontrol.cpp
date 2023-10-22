#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"


/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{

    con.ButtonUp.pressed([]()
                         { fw.spinRPM(1000); });
    con.ButtonLeft.pressed([]()
                           { fw.spinRPM(500); });
    con.ButtonRight.pressed([]()
                            { fw.spinRPM(750); });
    con.ButtonDown.pressed([]()
                           { fw.spinRPM(0); });

    while (imu.isCalibrating())
    {
        vexDelay(20);
    }

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
        vexDelay(10);
    }

    // ================ PERIODIC ================
}