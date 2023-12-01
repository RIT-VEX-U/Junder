#include "automation.h"
#include "robot-config.h"

// ================ Autonomous Abstractions ================


// ================ Driver Assist Automations ================

void matchload_1(vex::controller::button &button)
{
    // Determine which side with imu
    bool shootLeft = true;


    FunctionCommand *waitForIntake = new FunctionCommand([&](){
        return intake_watcher.objectDistance(distanceUnits::mm) < 150;
    });

    CommandController cmd{
        cata_sys.IntakeFully(),
        waitForIntake, 

    };
    
    // Cancel the operation if the button is ever released
    cmd.add_cancel_func([&](){return !button.pressing();});
    
    cmd.run();


}