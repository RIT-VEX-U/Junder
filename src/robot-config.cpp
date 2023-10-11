#include "robot-config.h"
vex::brain Brain;
vex::controller con;

#ifdef COMP_BOT

inertial imu(PORT10);

// 1 is frontmost, 4 is rearmost
vex::motor left1(vex::PORT11, vex::gearSetting::ratio6_1, true);
vex::motor left2(vex::PORT22, vex::gearSetting::ratio6_1, true);
vex::motor left3(vex::PORT13, vex::gearSetting::ratio6_1, true);
vex::motor left4(vex::PORT14, vex::gearSetting::ratio6_1, true);

vex::motor right1(vex::PORT15, vex::gearSetting::ratio6_1, false);
vex::motor right2(vex::PORT16, vex::gearSetting::ratio6_1, false);
vex::motor right3(vex::PORT17, vex::gearSetting::ratio6_1, false);
vex::motor right4(vex::PORT18, vex::gearSetting::ratio6_1, false);

vex::motor_group left_motors(left1, left2, left3, left4);
vex::motor_group right_motors(right1, right2, right3, right4);

vex::motor cata_r(vex::PORT4, vex::gearSetting::ratio36_1, false);
vex::motor cata_l(vex::PORT5, vex::gearSetting::ratio36_1, true);

vex::motor_group cata_motors(cata_l, cata_r);

std::map<std::string, vex::motor &> motor_names = {
    {"left 1", left1},
    {"left 2", left2},
    {"left 3", left3},
    {"left 4", left4},

    {"right 1", right1},
    {"right 2", right2},
    {"right 3", right3},
    {"right 4", right4},

    {"cata L", cata_l},
    {"cata R", cata_r},
};

PID::pid_config_t drive_pid_cfg =
    {
        .p = .2,
        .i = 0.0,
        .d = .02,
        .deadband = 0.1,
        .on_target_time = 0};

PID::pid_config_t turn_pid_cfg =
    {
        .p = 0.014,
        .i = 0.01,
        .d = 0.0025,
        .deadband = 1,
        .on_target_time = 0.1};

robot_specs_t robot_cfg = {
    .robot_radius = 12,            // inches
    .odom_wheel_diam = 4,          // inches
    .odom_gear_ratio = 0.5,        // inches
    .dist_between_wheels = 9.0,    // inches
    .drive_correction_cutoff = 12, // inches
    .drive_feedback = new PID(drive_pid_cfg),
    .turn_feedback = new PID(turn_pid_cfg),
    .correction_pid = (PID::pid_config_t){
        .p = .03,
    }};

OdometryTank odom{left_motors, right_motors, robot_cfg};
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

#else

// ================ INPUTS ================
// Digital sensors

// Analog sensors

// ================ OUTPUTS ================
// Motors

// ================ SUBSYSTEMS ================

inertial imu(PORT10);
CustomEncoder right_enc = CustomEncoder{Brain.ThreeWirePort.A, 90};
CustomEncoder left_enc = CustomEncoder{Brain.ThreeWirePort.C, 90};

// ======== SUBSYSTEMS ========
PID::pid_config_t drive_pid_cfg =
    {
        .p = .2,
        .i = 0.0,
        .d = .02,
        .deadband = 0.1,
        .on_target_time = 0};

PID::pid_config_t turn_pid_cfg =
    {
        .p = 0.014,
        .i = 0.01,
        .d = 0.0025,
        .deadband = 1,
        .on_target_time = 0.1};

robot_specs_t robot_cfg = {
    .robot_radius = 12,            // inches
    .odom_wheel_diam = 2.84,       // inches
    .odom_gear_ratio = 1.03,       // inches
    .dist_between_wheels = 9.18,   // inches
    .drive_correction_cutoff = 12, // inches
    .drive_feedback = new PID(drive_pid_cfg),
    .turn_feedback = new PID(turn_pid_cfg),
    .correction_pid = (PID::pid_config_t){
        .p = .03,
    }};

vex::motor left_front(vex::PORT1, true);
vex::motor left_back(vex::PORT2, true);

vex::motor right_front(vex::PORT3);
vex::motor right_back(vex::PORT4);

vex::motor_group left_motors(left_front, left_back);
vex::motor_group right_motors(right_front, right_back);

OdometryTank odom{left_enc, right_enc, robot_cfg, &imu};
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

// ================ UTILS ================

#endif

std::vector<screen::Page *> pages;
/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    pages = {new screen::StatsPage(motor_names), new screen::OdometryPage(odom, 12, 12)};
    odom.set_position({36, 36, 45});
    imu.calibrate();
    screen::start_screen(Brain.Screen, pages, 1);
}
