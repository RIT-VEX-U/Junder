#include "robot-config.h"
vex::brain Brain;
vex::controller con;

#ifdef COMP_BOT

inertial imu(PORT12);

vex::motor left_front(vex::PORT14, vex::gearSetting::ratio6_1, true);
vex::motor left_middle(vex::PORT20, vex::gearSetting::ratio6_1, true);
vex::motor left_back(vex::PORT15, vex::gearSetting::ratio6_1, true);
vex::motor left_raised(vex::PORT8, vex::gearSetting::ratio6_1, false);

vex::motor right_front(vex::PORT6, vex::gearSetting::ratio6_1, false);
vex::motor right_middle(vex::PORT3, vex::gearSetting::ratio6_1, false);
vex::motor right_back(vex::PORT11, vex::gearSetting::ratio6_1, false);
vex::motor right_raised(vex::PORT5, vex::gearSetting::ratio6_1, true);

vex::motor_group left_motors(left_front, left_middle, left_back, left_raised);
vex::motor_group right_motors(right_front, right_middle, right_back, right_raised);

vex::motor cata_r(vex::PORT4, vex::gearSetting::ratio36_1, false);
vex::motor cata_l(vex::PORT5, vex::gearSetting::ratio36_1, true);

vex::motor_group cata_motors(cata_l, cata_r);

std::map<std::string, vex::motor &> motor_names = {
    {"left f", left_front},
    {"left m", left_middle},
    {"left b", left_back},
    {"left r", left_raised},

    {"right f", right_front},
    {"right m", right_middle},
    {"right b", right_back},
    {"right r", right_raised},

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

    odom.set_position({36, 36, 45});
    pages = {new screen::StatsPage(motor_names), new screen::OdometryPage(odom, 12, 12, true)};
    imu.calibrate();
    screen::start_screen(Brain.Screen, pages, 1);
}
