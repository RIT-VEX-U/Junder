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
    while(true)
    {
        double l = static_cast<double>(con.Axis3.position()) / 100.0;
        double r = static_cast<double>(con.Axis2.position()) / 100.0;
        drive_sys.drive_tank(l, r, 1);

        vexDelay(10);
    }
}