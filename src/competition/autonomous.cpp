#include "competition/autonomous.h"
#include "automation.h"

#include <functional>

#include "core.h"
#include "robot-config.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev
#ifdef COMP_BOT

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
AutoCommand *printOdom = new FunctionCommand([]() {
    auto pose = odom.get_position();
    printf("(%.2f, %.2f) - %.2fdeg\n", pose.x, pose.y, pose.rot);
    return true;
});
pose_t gps_pose();
AutoCommand *recal = new FunctionCommand([]() {
    odom.set_position(gps_pose());
    return true;
});
/**
 * Main entrypoint for the autonomous period
 */
void only_shoot();
void supportMaximumTriballs();
void support_AWP();
void autonomous() {

    cata_sys.send_command(CataSys::Command::StartDropping);
    while (imu.isCalibrating() || gps_sensor.isCalibrating()) {
        vexDelay(20);
    }
    // vexDelay(2000);
    // support_AWP();
    // supportMaximumTriballs();
    only_shoot();
}

pose_t gps_pose() {
    while (gps_sensor.xPosition() == 0.0 && gps_sensor.yPosition() == 0.0) {
        vexDelay(20);
    }
    if (gps_sensor.quality() == 0) {
        return odom.get_position();
    }

    pose_t orig = odom.get_position();
    static timer t;
    t.reset();
    double x = orig.x, y = orig.y, rot = orig.rot;
    int itr = 0;
    while (t.time(sec) < 1) {
        if (gps_sensor.quality() > 99) {
            x += gps_sensor.xPosition(distanceUnits::in) + 72;
            y += gps_sensor.yPosition(distanceUnits::in) + 72;
            rot = gps_sensor.heading();
            itr++;
        }
    }

    if (itr > 0) {
        x = x / itr;
        y = y / itr;
    }

    pose_t pose;
    // double x = gps_sensor.xPosition(vex::distanceUnits::in) + 72.0;
    // double y = gps_sensor.yPosition(vex::distanceUnits::in) + 72.0;
    // double rot = gps_sensor.heading();
    if (!red_side) {
        x = 144 - x;
        y = 144 - y;
        rot += 180;
    }

    pose.x = x;
    pose.y = y;
    pose.rot = rot;
    printf("GPS: (%.2f, %.2f) %.2f\n", x, y, rot);
    return pose;
}
AutoCommand *get_and_score_alliance() {
    return new InOrder{
        // setup
        new FunctionCommand([]() {
            odom.set_position({.x = 28, .y = 18, .rot = 180});
            return true;
        }),

        // Drive to linup for alliance
        // drive_sys.DriveForwardCmd(5, FWD)->withTimeout(2.0),
        drive_sys.TurnDegreesCmd(-35),
        drive_sys.DriveForwardCmd(10, FWD)->withTimeout(2.0),
        drive_sys.TurnDegreesCmd(75),
        // recal,
        // drive_sys.TurnToHeadingCmd(232)->withTimeout(2.0),
        // printOdom,
        // recal,
        // Pickup Alliance
        cata_sys.IntakeToHold(),
        drive_sys.DriveTankCmd(0.2, 0.2)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.5))
            ->withTimeout(1.0),
        cata_sys.WaitForHold()->withTimeout(2.0),
        // recal,
        drive_sys.DriveForwardCmd(5.0, REV)->withTimeout(2.0),
        // Turn to side
        drive_sys.TurnToPointCmd(12.0, 36.0)->withTimeout(1.0),
        drive_sys.DriveForwardCmd(8.5, FWD, 0.4)->withTimeout(2.0),
        // recal,
        drive_sys.TurnToHeadingCmd(90.0)->withTimeout(1.0),
        drive_sys.DriveForwardCmd(6.5, FWD, 0.4)->withTimeout(2.0),
        // Dump in goal
        cata_sys.Unintake(),
        new DelayCommand(500),
        drive_sys.DriveForwardCmd(8.0, REV)->withTimeout(2.0),
        cata_sys.StopIntake(),

        drive_sys.TurnToHeadingCmd(0)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(282)->withTimeout(2.0),

        // Ram once
        drive_sys.DriveForwardCmd(20, REV)
            ->withTimeout(2.0)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.25)),
        drive_sys.DriveForwardCmd(4, FWD)->withTimeout(2.0),
        // Ram Twice
        drive_sys.DriveTankCmd(-0.5, -0.5)->withTimeout(0.5),
    };
}

void supportMaximumTriballs() {

    odom.set_position({.x = 28, .y = 18, .rot = 180});
    CommandController cc{

        get_and_score_alliance(),
        // Evacuate
        recal,
        drive_sys.DriveForwardCmd(4, FWD)->withTimeout(2.0),

        recal,
        // line up to load
        drive_sys.TurnToPointCmd(24, 26)->withTimeout(2.0),
        drive_sys.DriveToPointCmd({24, 26})->withTimeout(2.0),
        recal,
        // load
        drive_sys.TurnToPointCmd(0, 2)->withTimeout(2.0),
        cata_sys.IntakeToHold(),
        drive_sys.DriveToPointCmd({0, 2}, FWD, 0.3)->withTimeout(2.0),
        drive_sys.DriveForwardCmd(4, REV)->withTimeout(2.0),
        cata_sys.WaitForHold()->withTimeout(2.0),

        drive_sys.TurnToHeadingCmd(-35)->withTimeout(2.0),
        drive_sys.DriveToPointCmd({55, 12}, FWD, 0.75)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(0),
        cata_sys.Unintake(),
        drive_sys.DriveToPointCmd({22, 26}, REV, 0.6)->withTimeout(2.0),
        cata_sys.StopIntake(),
        // // Turn to pick up triball
        // drive_sys.TurnToHeadingCmd(-135),
        // cata_sys.IntakeToHold(),
        // drive_sys.DriveTankCmd(0.3, 0.3)->withTimeout(0.5),
        // cata_sys.WaitForHold()->withTimeout(2.0),

        // drive_sys.DriveForwardCmd(5, REV),
        new FunctionCommand([]() { return false; }),

    };
    cc.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cc.run();
}

void support_AWP() {

    CommandController cc{
        get_and_score_alliance(),
        // Evacuate
        recal,
        drive_sys.DriveForwardCmd(4, FWD)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(120),
        // Wall align
        drive_sys.DriveForwardCmd(24, REV)->withTimeout(2.0),
        drive_sys.DriveTankCmd(-0.5, -0.5)->withTimeout(1.5),
        drive_sys.DriveForwardCmd(4, FWD)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(0)->withTimeout(2.0),
        recal,
        drive_sys.DriveToPointCmd({50, 12})->withTimeout(2.0),
        // Turn to pole
        drive_sys.TurnToPointCmd(72, 24),
        recal,
        drive_sys.DriveToPointCmd({72, 24}, FWD, 0.4)->withTimeout(2.0),
        new FunctionCommand([]() { return false; }),

    };
    cc.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cc.run();
}

AutoCommand *shoot_and_drive(double dist, vex::directionType dir,
                             double max_pow) {

    return new InOrder{
        cata_sys.Fire(),
        drive_sys.DriveForwardCmd(dist, dir, max_pow)->withTimeout(1.5),
    };
}

void only_shoot() {
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
        return cata_watcher.isNearObject();
    });

    const double dist = 8.0;
    const double load_angle = 225;
    const double shoot_angle = 210.0;

    AutoCommand *printOdom = new FunctionCommand([]() {
        auto pose = odom.get_position();
        printf("(%.2f, %.2f) - %.2fdeg\n", pose.x, pose.y, pose.rot);
        return true;
    });
    printf("only shoot\n");

    CommandController cmd{
        odom.SetPositionCmd({.x = 22.0, .y = 22.0, .rot = 225}),
        new DelayCommand(300),
        printOdom,

        // 1 - Turn and shoot preload
        // cata_sys.Fire(),
        cata_sys.IntakeFully()->withTimeout(2.0),
        drive_sys.DriveForwardCmd(dist, FWD, 0.3)->withTimeout(1.0),

        // 2 - Turn to matchload zone & begin matchloading

        // Matchloading phase
        new RepeatUntil(
            InOrder{
                odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

                intakeToCata->withTimeout(1.75),

                drive_sys.DriveToPointCmd({24, 22}, REV, 0.4)->withTimeout(1.0),
                // ->withCancelCondition(
                // drive_sys.DriveStalledCondition(0.25)),
                cata_sys.Fire(),
                new DelayCommand(200),
                // },
                cata_sys.StopFiring()->withTimeout(0.5),

                cata_sys.IntakeFully()->withTimeout(2.0),
                drive_sys.TurnToHeadingCmd(load_angle, 0.5),

                // drive_sys.DriveForwardCmd(dist + 2, FWD, 0.2)
                // ->withTimeout(1.7)},
                drive_sys.DriveToPointCmd({13, 14}, FWD, 0.4)
                    ->withTimeout(1.0)
                    ->withCancelCondition(
                        drive_sys.DriveStalledCondition(0.25)),
            },
            new IfTimePassed(42)),

        drive_sys.DriveForwardCmd(3, REV),
        cata_sys.Fire(),
        new DelayCommand(200),
        cata_sys.StopIntake(),
        drive_sys.TurnToHeadingCmd(165)->withTimeout(2.0),
        drive_sys.DriveToPointCmd({50, 10}, REV)->withTimeout(5.0),
        printOdom,
        drive_sys.TurnToHeadingCmd(175)->withTimeout(2.0),
        drive_sys.DriveToPointCmd({100, 15}, REV)->withTimeout(5.0),
        // new WingCmd(true),
        drive_sys.TurnToHeadingCmd(-125)->withTimeout(3.0),
        // drive_sys.DriveForwardCmd(36.0, REV)->withTimeout(2.0),
        // drive_sys.DriveForwardCmd(30.0, FWD, 0.5)->withTimeout(2.0),
        drive_sys.DriveForwardCmd(36.0, REV)->withTimeout(2.0),
        drive_sys.DriveForwardCmd(18.0, FWD, 0.5)->withTimeout(2.0),
        drive_sys.TurnDegreesCmd(65)->withTimeout(2.0),
        drive_sys.DriveForwardCmd(24.0, REV)->withTimeout(2.0),
        drive_sys.TurnDegreesCmd(-120)->withTimeout(2.0),
        drive_sys.DriveForwardCmd(36.0, REV)->withTimeout(2.0),
        recal,
    };
    cmd.run();
}

void skills2() {
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

    auto thingRight = new FunctionCommand([]() {
        // Turn Right
        // disable_drive = true;
        right_motors.spin(directionType::rev, 5, volt);
        left_motors.spin(directionType::fwd, 3, volt);
        vexDelay(150);
        right_motors.stop(brakeType::hold);
        left_motors.stop(brakeType::hold);
        vexDelay(150);
        right_motors.stop(brakeType::coast);
        left_motors.stop(brakeType::coast);
        return true;
    });

    auto thingLeft = new FunctionCommand([]() {
        // Turn Left
        right_motors.spin(directionType::fwd, 3, volt);
        left_motors.spin(directionType::rev, 5, volt);
        vexDelay(150);
        right_motors.stop(brakeType::hold);
        left_motors.stop(brakeType::hold);
        vexDelay(150);
        right_motors.stop(brakeType::coast);
        left_motors.stop(brakeType::coast);
        return true;
    });

    CommandController cmd{
        odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

        // 1 - Turn and shoot preload
        // drive_sys.DriveForwardCmd(2.0, REV),
        // drive_sys.TurnToHeadingCmd(shoot_angle, .5), cata_sys.Fire(),
        // cata_sys.StopFiring(),

        // new DelayCommand(300),

        // 2 - Turn to matchload zone & begin matchloading
        // drive_sys.TurnToHeadingCmd(load_angle, .5),
        // cata_sys.IntakeFully(), drive_sys.DriveForwardCmd(dist + 2,
        // vex::fwd, 0.5)->withTimeout(1.5), thingLeft, Matchloading phase
        new RepeatUntil(
            InOrder{
                odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

                intakeToCata->withTimeout(2.0),
                // drive_sys.DriveForwardCmd(dist, REV, 0.5),
                // drive_sys.TurnToHeadingCmd(shoot_angle, 0.5),
                cata_sys.Fire(), new DelayCommand(300),
                // drive_sys.TurnToHeadingCmd(load_angle, 0.5),
                cata_sys.StopFiring(),

                cata_sys.IntakeFully()->withTimeout(5.0),
                // drive_sys.DriveForwardCmd(dist + 2, FWD, 0.5)
                // ->withTimeout(2.0)
                // ->withCancelCondition(
                // drive_sys.DriveStalledCondition(0.25)),
            },
            new IfTimePassed(45)),
        //
        // Last preload
        intakeToCata,
        drive_sys.DriveForwardCmd(dist, REV, 0.5)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(shoot_angle, 0.5), cata_sys.Fire(),
        new DelayCommand(500), drive_sys.TurnToHeadingCmd(load_angle, 0.5),
        drive_sys.DriveForwardCmd(dist + 2, FWD, 0.5)->withTimeout(4.0),

        cata_sys.StopFiring(),

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
                                             {.x = 30, .y = 11},
                                             {.x = 62, .y = 12},
                                             {.x = 100.0, .y = 19},
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
        drive_sys.DriveForwardCmd(34, FWD)->withTimeout(3.0),
        drive_sys.DriveForwardCmd(40, REV)->withTimeout(3.0),
        new WingCmd(RIGHT, false)};

    cmd.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cmd.run();

    drive_sys.stop();
}

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
        cata_sys.StopFiring(),

        new DelayCommand(300),

        // 2 - Turn to matchload zone & begin matchloading
        drive_sys.TurnToHeadingCmd(load_angle, .5), cata_sys.IntakeFully(),
        drive_sys.DriveForwardCmd(dist + 2, vex::fwd, 0.5)->withTimeout(1.5),

        // Matchloading phase
        new RepeatUntil(
            InOrder{
                odom.SetPositionCmd({.x = 16.0, .y = 16.0, .rot = 225}),

                intakeToCata->withTimeout(2.0),
                drive_sys.DriveForwardCmd(dist, REV, 0.5),
                drive_sys.TurnToHeadingCmd(shoot_angle, 0.5),
                cata_sys.Fire(),
                new DelayCommand(300),
                drive_sys.TurnToHeadingCmd(load_angle, 0.5),
                cata_sys.StopFiring(),

                cata_sys.IntakeFully(),
                drive_sys.DriveForwardCmd(dist + 2, FWD, 0.5)
                    ->withTimeout(2.0)
                    ->withCancelCondition(
                        drive_sys.DriveStalledCondition(0.25)),
            },
            new IfTimePassed(60)),
        //
        // Last preload
        intakeToCata,
        drive_sys.DriveForwardCmd(dist, REV, 0.5)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(shoot_angle, 0.5), cata_sys.Fire(),
        new DelayCommand(500), drive_sys.TurnToHeadingCmd(load_angle, 0.5),
        drive_sys.DriveForwardCmd(dist + 2, FWD, 0.5)->withTimeout(4.0),

        cata_sys.StopFiring(),

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
                                             {.x = 30, .y = 11},
                                             {.x = 62, .y = 12},
                                             {.x = 100.0, .y = 19},
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
        drive_sys.DriveForwardCmd(34, FWD)->withTimeout(3.0),
        drive_sys.DriveForwardCmd(40, REV)->withTimeout(3.0),
        new WingCmd(RIGHT, false)};

    cmd.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cmd.run();

    drive_sys.stop();
}

#else
void autonomous() {}
#endif