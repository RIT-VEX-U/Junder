#include "automation.h"
#include "robot-config.h"
#include "vision.h"

#define FWD vex::directionType::fwd
#define REV vex::directionType::rev

// ================ Autonomous Abstractions ================

// VISION TRACKING
vision_filter_s default_vision_filter = {
    .min_area = 300,
    .aspect_low = 0.8,
    .aspect_high = 4,

    .min_x = 0,
    .max_x = 320,
    .min_y = 0,
    .max_y = 240,
};

FeedForward::ff_config_t angle_ff_cfg{

};
PID::pid_config_t angle_pid_cfg{.p = 0.004, .d = 0.0005};

VisionTrackTriballCommand::VisionTrackTriballCommand(vision_filter_s &filter)
    : angle_fb(angle_pid_cfg, angle_ff_cfg), filter(filter) {}

bool VisionTrackTriballCommand::run() {
    static const int center_x = 160;
    static const double min_drive_speed = 0.1;
    static const double max_drive_speed = 0.3;

    static const double max_angle_speed = 0.3;
    static const double area_speed_scalar =
        50000; // Area at which speed is zero

    auto dist_from_area = [](double area) -> double {
        return 1284.0 * pow(area, -0.491);
    };
    auto ang_from_x = [](double x) -> double {
        return 53.9 + 0.241 * x + -3.05e-05 * (x * x);
    };

    std::vector<vision::object> sensed_obj = vision_run_filter(TRIBALL);

    if (sensed_obj.size() <= 0) {
        // Stop & wait if there isn't anything sensed
        drive_sys.stop();
        return false;
    }

    // if(intake_watcher.objectDistance(distanceUnits::mm) < 100)
    // {
    //     // Done when triball is in the intake
    //     drive_sys.stop();
    //     return true;
    // }

    // Get the largest object sensed
    vision::object largest;
    for (vision::object &obj : sensed_obj) {
        if ((obj.width * obj.height) > (largest.width * largest.height))
            largest = obj;
    }

    double object_area = largest.width * largest.height;

    angle_fb.set_target(center_x);
    angle_fb.update(largest.centerX);
    angle_fb.set_limits(-max_angle_speed, max_angle_speed);

    // Slow down as size of object increases (big area = small speed)
    // TODO test this
    // double speed = clamp(1-(area_speed_scalar * object_area), 0,
    // max_drive_speed);
    double speed = clamp(lerp(1, 0.0, object_area / area_speed_scalar), 0, 1) *
                       max_drive_speed +
                   min_drive_speed;
    // double speed = max_drive_speed;
    double heading = ang_from_x(largest.centerX);
    double dist = dist_from_area(object_area);
    printf("ang: %.2f  dist: %.2f\n", heading, dist);
    drive_sys.drive_tank_raw(speed + angle_fb.get(), speed - angle_fb.get());

    return false;
}

std::vector<vision::object> vision_run_filter(vision::signature &sig,
                                              vision_filter_s &filter) {
    cam.takeSnapshot(TRIBALL);
    vision::object &sensed = cam.objects[0];
    std::vector<vision::object> out;

    // Go through all sensed objects
    for (int i = 0; i < cam.objectCount; i++) {
        vision::object &cur_obj = cam.objects[i];

        // Filtering by size, aspect ratio & location in frame
        int area = cur_obj.width * cur_obj.height;
        double aspect_ratio = cur_obj.width / cur_obj.height;
        int x = cur_obj.centerX;
        int y = cur_obj.centerY;

        // keep searching if filtered
        if (area < filter.min_area || aspect_ratio < filter.aspect_low ||
            aspect_ratio > filter.aspect_high || x < filter.min_x ||
            x > filter.max_x || y < filter.min_y || y > filter.max_y) {
            continue;
        }

        out.push_back(cur_obj);
    }

    // Only for debugging

    return out;
}

VisionObjectExists::VisionObjectExists(vision_filter_s &filter)
    : filter(filter) {}

bool VisionObjectExists::test() {
    return vision_run_filter(TRIBALL, this->filter).size() > 0;
}

point_t estimate_triball_pos(vision::object &obj) {
    pose_t robot_pose = odom.get_position();

    double area = obj.width * obj.height;
    double x = 0; // TODO find formulae from spreadsheet
    double y = 0;

    // In reference to the camera
    point_t local_pos = {.x = x, .y = y};

    // Rotate in reference to the robot's heading
    point_t field_pos =
        (Mat2::FromRotationDegrees(robot_pose.rot) * local_pos) +
        robot_pose.get_point();

    return field_pos;
}

IsTriballInArea::IsTriballInArea(point_t pos, int radius,
                                 vision_filter_s &filter)
    : filter(filter), pos(pos), radius(radius) {}

bool IsTriballInArea::test() {
    std::vector<vision::object> objs = vision_run_filter(TRIBALL, filter);

    // Run through all objects sensed & return true if any is within a certain
    // distance
    for (int i = 0; i < objs.size(); i++) {
        point_t obj_pos = estimate_triball_pos(objs[i]);
        if (fabs(obj_pos.dist(pos)) <= radius)
            return true;
    }

    return false;
}

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
#ifdef COMP_BOT
    return new BasicSolenoidSet(climb_solenoid, true);
#else
    return new FunctionCommand([] { return true; });
#endif
}

AutoCommand *WingSetCmd(bool val) {
    return new FunctionCommand([val]() {
#ifdef COMP_BOT
        left_wing.set(val);
        right_wing.set(val);
#endif
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