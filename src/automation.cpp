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

    // CommandController cmd{
    // cata_sys.IntakeFully()->run();
    // while(!waitForIntake.run() && button) {vexDelay(10);}
    // while(!drive_sys.drive_forward(6, vex::directionType::rev, .5) && button ){ vexDelay(10); }
    // while(!drive_sys.turn_degrees(-10, .5) && button) { vexDelay(10); }
    // cata_sys.Fire()->run();
    // vexDelay(300);
    // while(!drive_sys.turn_degrees(10, .5) && button) {vexDelay(10);}
    // while(!drive_sys.drive_forward(6, vex::directionType::fwd, .5) && button) {vexDelay(10);}
        
    // };

}