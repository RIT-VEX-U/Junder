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
    RaiseLiftCommand(double inches){}
};

void autonomous()
{
    while (imu.isCalibrating())
    {
        vex::wait(0.02, vex::sec);
    }

    Awaitable lift1{new RaiseLiftCommand(12.0)};

    CommandController cc{
        lift1.start(),
        new DriveToPointCommand(drive_sys, *robot_cfg.turn_feedback, point_t{12.0, 0.0}, vex::forward),
        lift1.await(4),
        // new TurnDegreesCommand(drive_sys, *robot_cfg.turn_feedback, 90, 1.0),
        // new Branch{
        // new FunctionCondition([]()
        //   { return true; }),
        // new TurnDegreesCommand(drive_sys, *robot_cfg.turn_feedback, -90, 1.0),
        // new InOrder{
        // new DelayCommand(1000),
        // new TurnDegreesCommand(drive_sys, *robot_cfg.turn_feedback, 90, 1.0),
        // },
        // },

        new Parallel{
            new InOrder{
                new DelayCommand(500),
                new FunctionCommand([]()
                                    { printf("500 ms delay\n"); return true; }),
            },
            new InOrder{
                new DelayCommand(1000),
                new FunctionCommand([]()
                                    { printf("1000 ms delay\n"); return true; }),
            }

        },
    };

    cc.run();
}