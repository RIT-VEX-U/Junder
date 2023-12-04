#include "competition/autonomous.h"

#include <functional>

#include "core.h"
#include "robot-config.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

enum Side { LEFT, RIGHT };

class WingCmd : public AutoCommandBase {
   public:
    WingCmd(Side s, bool deploy_down) : s(s), deploy_down(deploy_down) {}

    bool run() override {
        if (s == LEFT) {
            if (deploy_down)
                left_wing.set(1);
            else
                left_wing.set(0);
        } else {
            if (deploy_down)
                right_wing.set(1);
            else
                right_wing.set(0);
        }
        return true;
    }

    AutoCommand duplicate() const override;

   private:
    Side s;
    bool deploy_down;
};

/**
 * Main entrypoint for the autonomous period
 */
void only_shoot();
void autonomous() {
    intake_combine.spinFor(directionType::rev, 1.0, timeUnits::sec, 100,
                           velocityUnits::pct);

    DONT_RUN_CATA_YOU_FOOL = false;

    only_shoot();
}

AutoCommand shoot_and_drive(double dist, vex::directionType dir,
                            double max_pow) {
    return InOrder{
        cata_sys.Fire(),
        drive_sys.DriveForwardCmd(dist, dir, max_pow).withTimeout(1.5),
    };
}

void only_shoot() {
    AutoCommand intakeToCata = FunctionCommand([]() {
        // Run intake in, periodically push out in case it's caught.
        // static vex::timer intake_tmr;
        // if(intake_tmr.time(sec) > 1)
        //     cata_sys.send_command(CataSys::Command::IntakeOut);
        // else if( intake_tmr.time(sec) > 1.5)
        //     intake_tmr.reset();
        // else
        //     cata_sys.send_command(CataSys::Command::IntakeIn);

        // Only return when the ball is in the bot
        return cata_watcher.isNearObject();
    });

    const double dist = 8.0;
    const double load_angle = 225;
    const double shoot_angle = 210.0;

    AutoCommand printOdom = FunctionCommand([]() {
        auto pose = odom.get_position();
        printf("(%.2f, %.2f) - %.2fdeg\n", pose.x, pose.y, pose.rot);
        return true;
    });
    printf("only shoot\n");

    CommandController cmd{
        odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),
        printOdom,

        // 1 - Turn and shoot preload
        cata_sys.Fire(),
        drive_sys.DriveForwardCmd(dist, REV),
        DelayCommand(300),
        cata_sys.StopFiring(),
        cata_sys.IntakeFully(),

        // 2 - Turn to matchload zone & begin matchloading
        drive_sys.DriveForwardCmd(dist + 2, vex::fwd, 0.5).withTimeout(1.5),

        // Matchloading phase
        Repeat{odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

               intakeToCata.withTimeout(1.75), cata_sys.Fire(),
               drive_sys.DriveForwardCmd(dist, REV, 0.5), cata_sys.StopFiring(),

               cata_sys.IntakeFully(),
               drive_sys.TurnToHeadingCmd(load_angle, 0.5),

               drive_sys.DriveForwardCmd(dist + 2, FWD, 0.2).withTimeout(1.7)},

        drive_sys.DriveForwardCmd(3, REV),
        cata_sys.Fire(),
        DelayCommand(300),
        cata_sys.StopIntake(),
        drive_sys.TurnToHeadingCmd(165).withTimeout(2.0),
        drive_sys
            .DriveToPointCmd(
                {
                    50,
                    10,
                },
                REV)
            .withTimeout(5.0),
        printOdom,
        drive_sys.TurnToHeadingCmd(175).withTimeout(2.0),
        drive_sys
            .DriveToPointCmd(
                {
                    100,
                    15,
                },
                REV)
            .withTimeout(5.0),
        drive_sys.TurnToHeadingCmd(-125).withTimeout(3.0),
        drive_sys.DriveForwardCmd(36.0, REV).withTimeout(2.0),
        drive_sys.DriveForwardCmd(30.0, FWD).withTimeout(2.0),
        drive_sys.DriveForwardCmd(36.0, REV).withTimeout(2.0),
        drive_sys.DriveForwardCmd(30.0, FWD).withTimeout(2.0),

    };
    cmd.run();
}

void skills() {
    AutoCommand intakeToCata = FunctionCommand([]() {
        // Only return when the ball is in the bot
        return cata_watcher.isNearObject();
    });

    const double dist = 6.0;
    const double load_angle = 225;
    const double shoot_angle = 210.0;

    CommandController cmd{
        odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

        // 1 - Turn and shoot preload
        {
            drive_sys.DriveForwardCmd(dist, REV),
            drive_sys.TurnToHeadingCmd(shoot_angle, .5),
            cata_sys.Fire(),
            cata_sys.StopFiring(),

            DelayCommand(300),
        },

        // 2 - Turn to matchload zone & begin matchloading
        {
            drive_sys.TurnToHeadingCmd(load_angle, .5),
            cata_sys.IntakeFully(),
            drive_sys.DriveForwardCmd(dist + 2, vex::fwd, 0.5).withTimeout(1.5),
        },
        // 3. Matchloading
        Repeat{
            odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

            intakeToCata.withTimeout(2.0),
            drive_sys.DriveForwardCmd(dist, REV, 0.5),
            drive_sys.TurnToHeadingCmd(shoot_angle, 0.5),
            cata_sys.Fire(),
            DelayCommand(300),
            drive_sys.TurnToHeadingCmd(load_angle, 0.5),
            cata_sys.StopFiring(),

            cata_sys.IntakeFully(),
            drive_sys.DriveForwardCmd(dist + 2, FWD, 0.5).withTimeout(2.0),
        }
            .withTimeout(4),

        // 4. Last preload
        {
            intakeToCata,
            drive_sys.DriveForwardCmd(dist, REV, 0.5).withTimeout(2.0),
            drive_sys.TurnToHeadingCmd(shoot_angle, 0.5),
            cata_sys.Fire(),
            DelayCommand(500),
            drive_sys.TurnToHeadingCmd(load_angle, 0.5),
            drive_sys.DriveForwardCmd(dist + 2, FWD, 0.5).withTimeout(4.0),

            cata_sys.StopFiring(),
        },

        // 5. Drive through "The Passage" & push into side of goal
        {
            odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),
            drive_sys.DriveToPointCmd({.x = 19, .y = 19}, REV, 0.5),
            drive_sys.TurnToHeadingCmd(160, .5),
            cata_sys.StopIntake(),
            drive_sys.TurnToHeadingCmd(60 - 180, .5),
            drive_sys.DriveForwardCmd(36, REV, 1).withTimeout(3.0),
            drive_sys.DriveForwardCmd(4, FWD, 1),
        },
        // 6. Wall align
        {
            drive_sys.TurnDegreesCmd(-80),
            drive_sys.DriveForwardCmd(8, REV, .15).withTimeout(3.0),
            odom.SetPositionCmd({.x = 137, .y = 51, .rot = 180}),
        },
        // 7. Drive out to front of goal and slam a couple times
        {drive_sys.DriveToPointCmd({.x = 90, .y = 51}, FWD, .5),
         WingCmd(RIGHT, true), drive_sys.TurnDegreesCmd(55),
         drive_sys.DriveForwardCmd(32, REV).withTimeout(3.0),
         drive_sys.TurnDegreesCmd(-10),
         drive_sys.DriveForwardCmd(34, FWD).withTimeout(3.0),
         drive_sys.DriveForwardCmd(40, REV).withTimeout(3.0),
         drive_sys.DriveForwardCmd(34, FWD).withTimeout(3.0),
         drive_sys.DriveForwardCmd(40, REV).withTimeout(3.0),
         WingCmd(RIGHT, false)},
    };

    cmd.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cmd.run();

    drive_sys.stop();
}