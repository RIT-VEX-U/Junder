#include "automation.h"
#include "robot-config.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

// ================ Autonomous Abstractions ================


// ================ Driver Assist Automations ================

void matchload_1(bool &enable)
{
    matchload_1([enable](){return enable;});
}

void matchload_1(std::function<bool()> enable)
{
    if(!enable())
        return;
        
    FunctionCommand *waitForIntake = new FunctionCommand([&](){
        return intake_watcher.objectDistance(distanceUnits::mm) < 150;
    });

    CommandController cmd{
        cata_sys.IntakeFully(),
        waitForIntake, 
        drive_sys.DriveForwardCmd(6, FWD, .5),
        drive_sys.TurnDegreesCmd(-10, .5),
        cata_sys.Fire(), 
        new DelayCommand(300),
        drive_sys.DriveForwardCmd(6, REV, .5),
        drive_sys.TurnDegreesCmd(10, .5),
        cata_sys.IntakeFully()
    };
    
    // Cancel the operation if the button is ever released
    cmd.add_cancel_func([&](){return !enable(); });
    
    cmd.run();
}