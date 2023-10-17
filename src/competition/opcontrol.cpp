#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{
    while (imu.isCalibrating())
    {
        vexDelay(20);
    }

    Condition * button_test = new FunctionCondition([](){return con.ButtonA.pressing();});
    CommandController c{
        new RepeatUntil(InOrder{
                            drive_sys.DriveToPointCmd({12, 0}, fwd, 0.2)->withCancelCondition(button_test),
                            drive_sys.DriveToPointCmd({0, 0}, reverse, 0.2),
                        },
                        (new TimesTestedCondition(4))->And(button_test)),
        // ->withTimeout(200),
        // new InOrder{
        // drive_sys.DriveForwardCmd(12, fwd, 0.2),
        // drive_sys.DriveForwardCmd(12, reverse, 0.2),
        // },
        // new RepeatUntil(,
        // 3)};
    };
    c.run();

    // return;
    // ================ INIT ================
    // con.ButtonL1.pressed([]()
    //  { cata_motors.spin(vex::fwd, 12.0, vex::voltageUnits::volt); });
    // con.ButtonL1.released([]()
    //   { cata_motors.stop(); });

    while (true)
    {
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        drive_sys.drive_tank(l, r);
        vexDelay(10);
    }

    // ================ PERIODIC ================
}