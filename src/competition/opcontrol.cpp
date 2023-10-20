#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    static vex::motor mot(vex::PORT1);

    static Feedback *tf = new PID(cfg);

    while (imu.isCalibrating())
    {
        vexDelay(20);
    }

    // ================ INIT ================
    const static double target_pos = 90.0;
    con.ButtonA.pressed([]()
                        { mot.setPosition(0, vex::degrees); tf->init(mot.position(vex::degrees), target_pos); });
    int tick = 0;
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
        if (tick % 2 == 0)
        {
            pos_graph.add_sample({(double)vex::timer::system(), pos});
            target_graph.add_sample({(double)vex::timer::system(), target_pos});
        }
        tf->update(pos);
        if (con.ButtonA.pressing())
        {
            mot.spin(vex::fwd, tf->get(), vex::volt);
        }
        else
            mot.stop();
        tick++;
        vexDelay(10);
    }

    // ================ PERIODIC ================
}