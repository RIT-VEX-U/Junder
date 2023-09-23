#include "competition/autonomous.h"
#include "robot-config.h"
#include "core.h"
#include <functional>
/**
 * Main entrypoint for the autonomous period
 */

class FunctionCommand : public AutoCommand
{
public:
    FunctionCommand(std::function<bool(void)> f) : f(f) {}
    bool run()
    {
        return f();
    }

private:
    std::function<bool(void)> f;
};

class RaiseLiftCommand : public AutoCommand
{
public:
    RaiseLiftCommand(double inches) {}
};

// static vex::motor_group left_motors{};
// Flywheel flywheel(left_motors, 1.0);

void autonomous()
{
    while (imu.isCalibrating())
    {
        vex::wait(0.02, vex::sec);
    }

    PID::pid_config_t pct = PID::pid_config_t{
        .p = .01,
    };

    PID p = PID(pct);

    CommandController cc{
        drive_sys.TurnDegreesCmd(90),
        drive_sys.DriveForwardCmd(10, vex::forward),
        drive_sys.TurnDegreesCmd(90),
        drive_sys.DriveForwardCmd(10),
        drive_sys.TurnDegreesCmd(90),
        drive_sys.DriveForwardCmd(10),
        drive_sys.TurnDegreesCmd(90),
        drive_sys.DriveForwardCmd(10),
        drive_sys.TurnDegreesCmd(90),
    };

    cc.run();
}