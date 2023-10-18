#include "competition/autonomous.h"
#include "robot-config.h"
#include "core.h"
#include <functional>
/**
 * Main entrypoint for the autonomous period
 */

vex::motor motor3{PORT3};

void autonomous()
{
    CommandController basic {
        new BasicSpinCommand(motor3, vex::directionType::fwd, BasicSpinCommand::type::percent, 100),
        new DelayCommand(1000),
        new BasicStopCommand(motor3, coast),

    };

    basic.run();
}