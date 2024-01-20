#include "automation.h"
#include "robot-config.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

// ================ Autonomous Abstractions ================

// ================ Driver Assist Automations ================

void matchload_1(std::function<bool()> enable) {
#ifdef COMP_BOT
    if (!enable())
        return;

    FunctionCommand *intakeToCata = new FunctionCommand([]() {
        drive_sys.drive_tank(0.15, 0.15);
        // Only return when the ball is in the bot
        return cata_watcher.isNearObject();
    });

    static timer drive_tmr;
    drive_tmr.reset();
    double rot = odom.get_position().rot;
    CommandController cmd{
        cata_sys.IntakeFully(),
        intakeToCata->withTimeout(3),
        new Async{new InOrder{
            new DelayCommand(100),
            cata_sys.Fire(),
        }},
        drive_sys.DriveForwardCmd(10, REV, 0.8)->withTimeout(1),
        drive_sys.TurnToHeadingCmd(rot - 2),
        new FunctionCommand([]() {
            cata_sys.send_command(CataSys::Command::StopFiring);
            return true;
        }),
        cata_sys.IntakeFully(),
        drive_sys.DriveForwardCmd(14, FWD, 0.2)->withTimeout(1),
    };

    // Cancel the operation if the button is ever released
    cmd.add_cancel_func([&]() { return !enable(); });
    cmd.run();
    cata_sys.send_command(CataSys::Command::StopIntake);
#endif
}

AutoCommand *ClimbBarDeploy() {
    return new BasicSolenoidSet(climb_solenoid, true);
}

AutoCommand *WingSetCmd(bool val) {
    return new FunctionCommand([val]() {
        left_wing.set(val);
        right_wing.set(val);
        return true;
    });
}

AutoCommand *GetOverBar() {
    double roll_target = -8.0;
    double over_bar_target = 0.0;
    double drive_amt = -0.8;
    return (new FunctionCommand(
                [=, flipped_up = false, over_bar = false]() mutable {
                    if (!flipped_up && imu.roll() < roll_target) {
                        flipped_up = true;
                    }
                    if (flipped_up) {
                        if (imu.roll() > over_bar_target) {
                            over_bar = true;
                        }
                    }
                    printf("%.2f - %d - %d\n", imu.roll(), (int)flipped_up,
                           (int)over_bar);
                    if (!over_bar) {
                        drive_sys.drive_tank(drive_amt, drive_amt);
                    } else {
                        drive_sys.stop();
                        return true;
                    }
                    return false;
                }))
        ->withTimeout(10.0);
}

AutoCommand *Climb() {
    double wall_align_pwr = -.3;
    double pipe_climb_power = -.6;
    return new InOrder{

        // Align with wall
        drive_sys.DriveTankCmd(wall_align_pwr, wall_align_pwr)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.5))
            ->withTimeout(5.0),

        // Climb up
        ClimbBarDeploy(),
        GetOverBar(),
        WingSetCmd(true),

        // Center for safety
        drive_sys.DriveForwardCmd(1.0),

    };
}