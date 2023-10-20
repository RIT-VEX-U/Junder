#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    imu.calibrate();
    // while(imu.isCalibrating()) {};
    
    // ================ INIT ================
    #ifdef COMP_BOT
    con.ButtonL1.pressed([](){cata_motors.spin(vex::fwd, 12.0, vex::voltageUnits::volt);});
    con.ButtonL1.released([](){cata_motors.stop();});
    #endif

    while (true)
    {
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        drive_sys.drive_tank(l, r);

        // printf("x: %f, y: %f, z: %f\n", 
            // imu.acceleration(xaxis), imu.acceleration(yaxis), imu.acceleration(zaxis));

        vexDelay(10);
    }

    // ================ PERIODIC ================
}