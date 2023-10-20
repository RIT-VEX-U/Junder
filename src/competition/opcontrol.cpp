#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    static vex::motor mot(vex::PORT1);


    while (imu.isCalibrating())
    {
        vexDelay(20);
    }

    // ================ INIT ================
    const static double target_pos = 90.0;
    con.ButtonA.pressed([]()
                        { mot.setPosition(0, vex::degrees); pid_to_tune->init(mot.position(vex::degrees), target_pos); });
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
        double pos = mot.position(vex::degrees);
        pid_to_tune->update(pos);
        if (con.ButtonA.pressing())
            mot.spin(vex::fwd, pid_to_tune->get(), vex::volt);
        else
            mot.stop();
        vexDelay(10);
    }

    // ================ PERIODIC ================
}