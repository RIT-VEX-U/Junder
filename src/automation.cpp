#include "automation.h"
#include "robot-config.h"

// ================ Autonomous Abstractions ================

// ================ Driver Assist Automations ================

void matchload_1(vex::controller::button& button)
{
    // Determine which side with imu
    bool shootLeft = true;


    CommandController cmd{
        cata_sys.IntakeFully(),
        cata_sys.WaitForIntake(),

    };

    // Cancel the operation if the button is ever released
    cmd.add_cancel_func([&]()
        { return !button.pressing(); });

    cmd.run();
}