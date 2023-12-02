#include "competition/autonomous.h"

#include <functional>

#include "core.h"
#include "robot-config.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

enum Side { LEFT, RIGHT };

class WingCmd : public AutoCommand {
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

    Side s;
    bool deploy_down;
};

/**
 * Main entrypoint for the autonomous period
 */

void autonomous() {}

void skills() {
    FunctionCommand *intakeToCata = new FunctionCommand([]() {
        // Run intake in, periodically push out in case it's caught.
        // static vex::timer intake_tmr;
        // if(intake_tmr.time(sec) > 1)
        //     cata_sys.send_command(CataSys::Command::IntakeOut);
        // else if( intake_tmr.time(sec) > 1.5)
        //     intake_tmr.reset();
        // else
        //     cata_sys.send_command(CataSys::Command::IntakeIn);

        // Only return when the ball is in the bot
        printf("intook: %d\n", (int)cata_watcher.isNearObject());
        return cata_watcher.isNearObject();
    });

    const double dist = 6.0;
    const double load_angle = 225;
    const double shoot_angle = 210.0;

    AutoCommand *printOdom = new FunctionCommand([]() {
        auto pose = odom.get_position();
        printf("(%.2f, %.2f) - %.2fdeg\n", pose.x, pose.y, pose.rot);
        return true;
    });

    CommandController cmd{
        odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

        // 1 - Turn and shoot preload
        drive_sys.DriveForwardCmd(dist, REV),
        drive_sys.TurnToHeadingCmd(shoot_angle, .5), cata_sys.Fire(),
        new DelayCommand(300),

        // 2 - Turn to matchload zone & begin matchloading
        drive_sys.TurnToHeadingCmd(load_angle, .5), cata_sys.IntakeFully(),
        drive_sys.DriveForwardCmd(dist + 1.5, vex::fwd, 0.5)->withTimeout(1.5),

        // Matchloading phase
        new RepeatUntil(
            InOrder{odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

                    intakeToCata->withTimeout(2.0),
                    drive_sys.DriveForwardCmd(dist, REV, 0.5),
                    drive_sys.TurnToHeadingCmd(shoot_angle, 0.5),
                    cata_sys.Fire(), new DelayCommand(300),
                    drive_sys.TurnToHeadingCmd(load_angle, 0.5),
                    cata_sys.IntakeFully(),
                    drive_sys.DriveForwardCmd(dist + 1, FWD, 0.5)
                        ->withTimeout(4.0)},
            new IfTimePassed(30)),
        //
        // Last preload
        intakeToCata, drive_sys.DriveForwardCmd(dist, REV, 0.5),
        drive_sys.TurnToHeadingCmd(shoot_angle, 0.5), cata_sys.Fire(),
        new DelayCommand(500), drive_sys.TurnToHeadingCmd(load_angle, 0.5),
        drive_sys.DriveForwardCmd(dist + 1, FWD, 0.5)->withTimeout(4.0),

        new FunctionCommand([]() {
            cata_sys.send_command(CataSys::Command::StopFiring);
            return true;
        }),

        printOdom, odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),
        drive_sys.DriveToPointCmd({.x = 19, .y = 19}, REV, 0.5), printOdom,
        drive_sys.TurnToHeadingCmd(160, .5), cata_sys.StopIntake(),

        // // Drive through "The Passage" & push into side of goal
        // // Push into side of goal
        new Parallel{
            new InOrder{new WaitUntilCondition(new FunctionCondition(
                            []() { return odom.get_position().x > 102; })),
                        new WingCmd(LEFT, false)},
            drive_sys.PurePursuitCmd(PurePursuit::Path(
                                         {
                                             {.x = 19, .y = 19},
                                             {.x = 30, 0, .y = 11},
                                             {.x = 62, .y = 13},
                                             {.x = 100.0, .y = 21},
                                             {.x = 120.0, .y = 26},
                                         },
                                         4),
                                     REV, .5)},
        drive_sys.TurnToHeadingCmd(60 - 180, .5),
        drive_sys.DriveForwardCmd(36, REV, 1)->withTimeout(3.0),
        drive_sys.DriveForwardCmd(4, FWD, 1), drive_sys.TurnDegreesCmd(-80),
        drive_sys.DriveForwardCmd(8, REV, .15)->withTimeout(3.0), // wall align
        printOdom,
        odom.SetPositionCmd(
            {.x = 137, .y = 51, .rot = 180}), // odom.get_position().rot
        printOdom, drive_sys.DriveToPointCmd({.x = 90, .y = 51}, FWD, .5),
        new WingCmd(RIGHT, true),
        // new WingCmd(LEFT, true),
        drive_sys.TurnDegreesCmd(55),
        drive_sys.DriveForwardCmd(32, REV)->withTimeout(3.0),
        drive_sys.TurnDegreesCmd(-10),
        drive_sys.DriveForwardCmd(34, FWD)->withTimeout(3.0),
        drive_sys.DriveForwardCmd(40, REV)->withTimeout(3.0),
        drive_sys.DriveForwardCmd(8, FWD)->withTimeout(3.0),
        // // Undeploy wing and back up, deploy other wing
        // new WingCmd(LEFT, false),
        // drive_sys.DriveToPointCmd({.x = 0, .y = 0}, FWD, 0.5),
        // drive_sys.TurnToHeadingCmd(0, 0.5), new WingCmd(RIGHT, true),

        // // Pure pursuit to center goal
        // drive_sys.PurePursuitCmd(PurePursuit::Path(
        //                              {
        //                                  {.x = 0, .y = 0},
        //                                  {.x = 0, .y = 0},
        //                                  {.x = 0, .y = 0},
        //                              },
        //                              12.0),
        //                          REV, 0.5),
        // drive_sys.DriveToPointCmd({.x = 0, .y = 0}, FWD, 0.5),
        new WingCmd(RIGHT, false)};

    cmd.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cmd.run();

    drive_sys.stop();
}