#include "competition/autonomous.h"
#include "automation.h"
#include <functional>
#include <memory>

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

/**
 * Main entrypoint for the autonomous period
 */
void supportMaximumTriballs();
void support_AWP();
void newMaxSkills();
void autonomous() {

    cata_sys.send_command(CataSys::Command::StartDropping);

    while (imu.isCalibrating() || gps_sensor.isCalibrating()) {
        vexDelay(20);
    }

    // supportMaximumTriballs();
    newMaxSkills();
}

AutoCommand *get_and_score_alliance() {
    return new InOrder{
        // setup
        new FunctionCommand([]() {
            odom.set_position({.x = 28, .y = 18, .rot = 180});
            return true;
        }),

        // Drive to linup for alliance
        drive_sys.TurnDegreesCmd(-35)->withTimeout(1.0),
        drive_sys.DriveForwardCmd(10, FWD)->withTimeout(1.0),
        drive_sys.TurnDegreesCmd(75)->withTimeout(1.0),

        // Pickup Alliance
        cata_sys.IntakeToHold(),
        drive_sys.DriveTankCmd(0.2, 0.2)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.5))
            ->withTimeout(1.0),
        cata_sys.WaitForHold()->withTimeout(2.0),
        drive_sys.DriveForwardCmd(5.0, REV)->withTimeout(1.0),

        // Turn to side
        drive_sys.TurnToPointCmd(9.0, 36.0)->withTimeout(1.0),
        drive_sys.DriveForwardCmd(8.5, FWD, 0.4)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(90.0)->withTimeout(1.0),
        drive_sys.DriveForwardCmd(7.0, FWD, 0.4)->withTimeout(2.0),

        // Dump in goal
        cata_sys.Unintake(),
        new DelayCommand(200),
        drive_sys.DriveForwardCmd(6.0, REV)->withTimeout(2.0),
        cata_sys.StopIntake(),

        drive_sys.TurnToHeadingCmd(0)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(282)->withTimeout(2.0),

        // Ram once
        drive_sys.DriveForwardCmd(20, REV)
            ->withTimeout(2.0)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.25)),
    };
}

void supportMaximumTriballs() {

    FieldSide fs = FieldSide::BLUE;
    if (red_side) {
        fs = FieldSide::RED;
    }

    odom.set_position({.x = 28, .y = 18, .rot = 180});
    CommandController cc{

        get_and_score_alliance(),
        // Evacuate
        drive_sys.DriveForwardCmd(4, FWD)->withTimeout(2.0),
        printOdom,
        // line up to load
        drive_sys.TurnToPointCmd(24, 26)->withTimeout(2.0),
        drive_sys.DriveToPointCmd({24, 26})->withTimeout(2.0),

        // load
        new RepeatUntil(
            InOrder{
                drive_sys.TurnToPointCmd(0, 0)->withTimeout(2.0),
                cata_sys.IntakeToHold(),
                drive_sys.DriveToPointCmd({0, 0}, FWD, 0.3)->withTimeout(1.0),
                odom.SetPositionCmd({.x = 18, .y = 18, .rot = 225}),
                // ->withCancelCondition(drive_sys.DriveStalledCondition(0.5)),

                drive_sys.DriveForwardCmd(4, REV)->withTimeout(1.0),
                cata_sys.WaitForHold()->withTimeout(1.0),

                /// Deliver the deliverables
                drive_sys.TurnToHeadingCmd(-35)->withTimeout(1.0),

                cata_sys.Unintake(),
                drive_sys
                    .PurePursuitCmd(PurePursuit::Path(
                                        {
                                            {36, 18},
                                            {39, 16},
                                        },
                                        4.0),
                                    FWD, 0.5)
                    ->withTimeout(2.0),

                // drive_sys.TurnToHeadingCmd(0)->withTimeout(2.0),

                drive_sys
                    .PurePursuitCmd(PurePursuit::Path(
                                        {
                                            {39, 16},
                                            {36, 18},
                                            {24, 24},
                                        },
                                        4.0),
                                    REV, 0.5)
                    ->withTimeout(2.0),

                cata_sys.StopIntake(),
            },
            new IfTimePassed(31)),
        drive_sys.TurnToPointCmd(0, 0)->withTimeout(2.0),
        new GPSLocalizeCommand(fs),
        cata_sys.IntakeToHold(),
        drive_sys.DriveToPointCmd({0, 0}, FWD, 0.3)->withTimeout(2.0),
        drive_sys.DriveForwardCmd(4, REV)->withTimeout(2.0),
        cata_sys.WaitForHold()->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(-35)->withTimeout(2.0),

        cata_sys.Unintake(),
        drive_sys.PurePursuitCmd(PurePursuit::Path(
                                     {
                                         {36, 14},
                                         {55, 12},
                                     },
                                     4.0),
                                 FWD, 0.5),
        drive_sys.TurnToHeadingCmd(0)->withTimeout(2.0),

        drive_sys.DriveForwardCmd(48.0, FWD, 0.5)->withTimeout(2.0),
        drive_sys.DriveForwardCmd(37.0, REV, 0.5)->withTimeout(2.0),
        cata_sys.StopIntake(),
        drive_sys.TurnDegreesCmd(45),
        drive_sys.DriveForwardCmd(12.0, FWD, 0.5)->withTimeout(5.0),

    };
    cc.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cc.run();
}

void support_AWP() {

    CommandController cc{
        get_and_score_alliance(),
        // Evacuate
        drive_sys.DriveForwardCmd(4, FWD)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(120),
        // Wall align
        drive_sys.DriveForwardCmd(24, REV)->withTimeout(2.0),
        drive_sys.DriveTankCmd(-0.5, -0.5)->withTimeout(1.5),
        drive_sys.DriveForwardCmd(4, FWD)->withTimeout(2.0),
        drive_sys.TurnToHeadingCmd(0)->withTimeout(2.0),
        drive_sys.DriveToPointCmd({50, 12})->withTimeout(2.0),
        // Turn to pole
        drive_sys.TurnToPointCmd(72, 24),
        drive_sys.DriveToPointCmd({72, 24}, FWD, 0.4)->withTimeout(2.0),
        new FunctionCommand([]() { return false; }),

    };
    cc.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cc.run();
}

void newMaxSkills() {
    FunctionCommand *intakeToCata =
        new FunctionCommand([]() { return cata_watcher.isNearObject(); });

    AutoCommand *printOdom = new FunctionCommand([]() {
        auto pose = odom.get_position();
        printf("(%.2f, %.2f) - %.2fdeg\n", pose.x, pose.y, pose.rot);
        return true;
    });

    FunctionCommand *tempend = new FunctionCommand([]() {
        drive_sys.stop();
        cata_sys.send_command(CataSys::Command::StopIntake);
        while (true) {
            cata_sys.send_command(CataSys::Command::StopIntake);
            double f = con.Axis3.position() / 200.0;
            double s = con.Axis1.position() / 200.0;
            drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
            pose_t pos = odom.get_position();
            printf("X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            vexDelay(100);
        }
        return false;
    });

    printf("hi");

    CommandController cmd{
        // set odom start pos
        odom.SetPositionCmd({.x = 22.0, .y = 22.0, .rot = 225}),
        new DelayCommand(900),
        printOdom,

        // backup into match loading
        cata_sys.IntakeFully()->withTimeout(1.0),
        drive_sys.DriveForwardCmd(10.0, FWD, 0.25)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.2))
            ->withTimeout(1.0),

        // match load for 42 seconds
        new RepeatUntil(
            InOrder{
                odom.SetPositionCmd({.x = 17.0, .y = 17.0, .rot = 225}),

                cata_sys.IntakeFully(),
                intakeToCata->withTimeout(1.0),

                // cata_sys.StopIntake()->withTimeout(2.0),

                drive_sys.DriveForwardCmd(7.0, REV, 0.5),

                // drive_sys.TurnToHeadingCmd()

                drive_sys.TurnToHeadingCmd(190)->withTimeout(1.0),

                // drive_sys.DriveForwardCmd(4.0, REV, 0.3)->withTimeout(1.0),

                cata_sys.Fire()->withTimeout(1.5),

                new DelayCommand(200),

                // cata_sys.StopFiring(),

                // cata_sys.IntakeFully()->withTimeout(0.5),

                // drive_sys.DriveForwardCmd(4.0, FWD, 0.3)->withTimeout(1.0),

                drive_sys.TurnToHeadingCmd(225)->withTimeout(1.0),

                drive_sys.DriveForwardCmd(10.0, FWD, 0.25)
                    ->withCancelCondition(drive_sys.DriveStalledCondition(0.25))
                    ->withTimeout(1.0),

            },
            new IfTimePassed(10)),

        // cata_sys.Fire()->withTimeout(1.0),

        // new DelayCommand(200),

        odom.SetPositionCmd({.x = 17.0, .y = 17.0, .rot = 225}),

        cata_sys.StopIntake()->withTimeout(1.5),

        new Async(new InOrder{
            new WaitUntilCondition(new FunctionCondition(
                []() { return odom.get_position().x > 110; })),
            new WingCmd(RIGHT, true),
            new WaitUntilCondition(new FunctionCondition(
                []() { return odom.get_position().y > 40; })),
            new WingCmd(RIGHT, false),
        }),

        // drive_sys.DriveToPointCmd({.x=110, .y=22}, REV, 0.3),
        drive_sys
            .PurePursuitCmd(PurePursuit::Path(
                                {
                                    {.x = 25, .y = 17},
                                    {.x = 55, .y = 17},
                                    {.x = 95, .y = 17},
                                    {.x = 110, .y = 20},
                                    {.x = 132, .y = 38},
                                    {.x = 145, .y = 48},
                                    // {.x=109, .y=66},
                                    // {.x=109, .y=78}
                                },
                                7),
                            REV, 0.4)
            ->withTimeout(4.0),

        // drive_sys.TurnToHeadingCmd(180)->withTimeout(1.0),

        drive_sys.TurnToHeadingCmd(270)->withTimeout(1.0),

        drive_sys.DriveForwardCmd(20, REV)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.3))
            ->withTimeout(1.5),

        drive_sys.DriveForwardCmd(10, FWD, 0.3)->withTimeout(1.0),

        drive_sys.DriveForwardCmd(20, REV)
            ->withCancelCondition(drive_sys.DriveStalledCondition(0.3))
            ->withTimeout(1.5),

        drive_sys.DriveForwardCmd(10, FWD, 0.3)->withTimeout(1.0),

        // drive_sys.PurePursuitCmd(PurePursuit::Path({
        //     {.x=132, .y=38},
        //     {.x=109, .y=32},
        //     {.x=101, .y=47},
        //     {.x=100, .y=69},
        // }, 7), FWD, 0.3)->withTimeout(4.0),

        // drive_sys.TurnToHeadingCmd(0),

        // drive_sys.DriveForwardCmd(20, FWD)
        //     ->withCancelCondition(drive_sys.DriveStalledCondition(0.3))
        //     ->withTimeout(1.5),

        // odom.set_position({.x=})

        tempend,

    };

    cmd.add_cancel_func([]() { return con.ButtonA.pressing(); });
    cmd.run();
}

#else
void autonomous() {}
#endif