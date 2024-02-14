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

// ================ GPS Localizing Functions ================

#define NUM_DATAPOINTS 100
#define GPS_GATHER_SEC 1.0

std::vector<pose_t> gps_gather_data() {
    std::vector<pose_t> pose_list;
    vex::timer tmr;

    // for(int i = 0; i < NUM_DATAPOINTS; i++)
    while (tmr.time(sec) < GPS_GATHER_SEC) {
        pose_t cur;
        cur.x = gps_sensor.xPosition(distanceUnits::in) + 72;
        cur.y = gps_sensor.yPosition(distanceUnits::in) + 72;
        cur.rot = gps_sensor.heading(rotationUnits::deg);

        pose_list.push_back(cur);
        vexDelay(1);
    }

    return pose_list;
}

void sort_by_distance_to_origin(std::vector<pose_t> &pose_list) {
    std::sort(pose_list.begin(), pose_list.end(), [](pose_t a, pose_t b) {
        point_t origin = {0, 0};
        return a.get_point().dist(origin) > b.get_point().dist(origin);
    });
}

pose_t get_pose_avg(std::vector<pose_t> pose_list) {
    // Get average point
    point_t avg_filtered = {0, 0};
    Vector2D heading_filtered(point_t{0, 0});
    for (pose_t p : pose_list) {
        avg_filtered.x += p.x;
        avg_filtered.y += p.y;

        heading_filtered = heading_filtered + Vector2D(deg2rad(p.rot), 1);
    }

    avg_filtered.x /= pose_list.size();
    avg_filtered.y /= pose_list.size();

    pose_t avg = {.x = avg_filtered.x,
                  .y = avg_filtered.y,
                  .rot = rad2deg(heading_filtered.get_dir())};

    return avg;
}

void gps_localize_median() {
    auto pose_list = gps_gather_data();
    sort_by_distance_to_origin(pose_list);

    pose_t median;
    if (pose_list.size() > 0) {
        median = pose_list[pose_list.size() / 2];
        odom.set_position(median);
    }

    printf("MEDIAN {%.2f, %.2f, %.2f}\n", median.x, median.y, median.rot);
}

std::tuple<pose_t, double> gps_localize_stdev() {
    auto pose_list = gps_gather_data();

    pose_t avg_unfiltered = get_pose_avg(pose_list);

    // Create a parallel list with corresponding distances
    std::vector<double> dist_list;
    for (pose_t p : pose_list)
        dist_list.push_back(p.get_point().dist({0, 0}));

    // Calculate standard deviation of distances to origin
    double dist_mean = mean(dist_list);
    double dist_stdev = sqrt(variance(dist_list, dist_mean));

    // Filter out points that are greater than 3 standard deviations away from
    // original mean
    for (int i = 0; i < pose_list.size(); i++) {
        if (fabs(dist_mean - dist_list[i]) > 1 * dist_stdev) {
            pose_list[i] = {-1, -1, -1}; // Mark as bad
        }
    }

    // Final removal of points
    auto itr = std::remove_if(pose_list.begin(), pose_list.end(), [](pose_t p) {
        return p.x == -1.0 && p.y == -1.0 && p.rot == -1.0;
    });
    // ACTUALLY remove it cause remove_if kinda sucks
    pose_list.erase(itr, pose_list.end());

    pose_t avg_filtered = get_pose_avg(pose_list);
    printf("Stddev: %f Mean: %f #Unfiltered: %lu #Filtered: %lu\n", dist_stdev,
           dist_mean, dist_list.size(), pose_list.size());
    printf("Unfiltered X: %f, Y: %f, H: %f\n", avg_unfiltered.x,
           avg_unfiltered.y, avg_unfiltered.rot);
    printf("Filtered X: %f, Y: %f, H: %f\n", avg_filtered.x, avg_filtered.y,
           avg_filtered.rot);

    return std::tuple<pose_t, double>(avg_filtered, dist_stdev);
}

bool GPSLocalizeCommand::first_run = true;
int GPSLocalizeCommand::rotation = 0;
const int GPSLocalizeCommand::min_rotation_radius = 48;
bool GPSLocalizeCommand::run() {
    // pose_t odom_pose = odom.get_position();
    auto [new_pose, stddev] = gps_localize_stdev();

    if (!red_side) {
        new_pose.x = 144 - new_pose.x;
        new_pose.y = 144 - new_pose.y;
        new_pose.rot += 180;
    }

    odom.set_position(new_pose);

    // Vector2D centrefield_gps(point_t{new_pose.x - 72, new_pose.y - 72});
    // Vector2D centerfield_odom(point_t{.x=odom_pose.x - 72, .y=odom_pose.y -
    // 72});

    // On the first localize, decide if the orientation of the field is correct.
    // If not, create a rotation to correct
    // if(first_run)
    // {

    //     for(int i = 0; i < 4; i++)
    //     {
    //         Vector2D rot(centrefield_gps.get_dir() + deg2rad(i * 90),
    //         centrefield_gps.get_mag()); printf("dist: %f\n",
    //         centerfield_odom.point().dist(rot.point())); printf("{%f, %f}\n",
    //         rot.get_x(), rot.get_y());
    //         // Test if the rotated vector is within an acceptable distance
    //         // to what odometry is reporting
    //         if (centerfield_odom.point().dist(rot.point()) <
    //         min_rotation_radius)
    //         {
    //             rotation = i * 90;
    //             break;
    //         }
    //     }

    //     printf("Localize init complete: Detected field rotated by %d
    //     degrees\n", rotation); first_run = false;
    // }

    // Vector2D rot(centrefield_gps.get_dir() + deg2rad(rotation),
    // centrefield_gps.get_mag()); odom.set_position(pose_t{
    //     .x = rot.get_x() + 72,
    //     .y = rot.get_y() + 72,
    //     .rot = new_pose.rot
    // });
    printf("Localized with variance of %f to {%f, %f, %f}\n", stddev,
           new_pose.x, new_pose.y, new_pose.rot);
    return true;
}

pose_t GPSLocalizeCommand::get_pose_rotated() {
    Vector2D new_pose_vec(
        point_t{.x = gps_sensor.xPosition(distanceUnits::in) + 72,
                .y = gps_sensor.yPosition(distanceUnits::in) + 72});
    Vector2D rot(new_pose_vec.get_dir() + deg2rad(rotation),
                 new_pose_vec.get_mag());

    return pose_t{.x = rot.get_x(),
                  .y = rot.get_y(),
                  .rot = gps_sensor.heading(rotationUnits::deg)};
}