#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{
    // ================ INIT ================

    printf("Going 👍\n");
    // ================ PERIODIC ================
    double deadband = 0.1;
    while(true)
    {
        double l = static_cast<double>(con.Axis3.position()) / 100.0;
        double r = static_cast<double>(con.Axis2.position()) / 100.0;
        if (fabs(l)>deadband || fabs(r) > deadband){
            drive_sys.drive_tank(l, r, 1);
        } else {
            drive_sys.drive_tank(deadband, deadband);
        }

        vexDelay(10);
    }
}