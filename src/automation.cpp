#include "automation.h"
#include "robot-config.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

// ================ Autonomous Abstractions ================

// ================ Driver Assist Automations ================

void matchload_1(std::function<bool()> enable) {
    if (!enable())
        return;

    AutoCommand intakeToCata = FunctionCommand([]() {
        drive_sys.drive_tank(0.15, 0.15);
        // Only return when the ball is in the bot
        return cata_watcher.isNearObject();
    });

    static timer drive_tmr;
    drive_tmr.reset();
    double rot = odom.get_position().rot;
    CommandController cmd{
        cata_sys.IntakeFully(),
        intakeToCata.with_timeout(3),
        drive_sys.DriveForwardCmd(10, REV, 0.8).with_timeout(1),
        drive_sys.TurnToHeadingCmd(rot - 2),
        FunctionCommand([]() {
            cata_sys.send_command(CataSys::Command::StopFiring);
            return true;
        }),
        cata_sys.IntakeFully(),
        drive_sys.DriveForwardCmd(14, FWD, 0.2).with_timeout(1),
    };

    // Cancel the operation if the button is ever released
    // cmd.add_cancel_func([&]() { return !enable(); });
    cmd.run();
    cata_sys.send_command(CataSys::Command::StopIntake);
}