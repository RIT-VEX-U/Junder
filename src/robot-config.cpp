#include "robot-config.h"
brain Brain;
controller con;

using namespace vex;

#ifdef COMP_BOT

// ================ INPUTS ================
// Digital sensors

// Analog sensors
inertial imu(PORT12);

// ================ OUTPUTS ================
// Motors
motor left_front(PORT14, gearSetting::ratio6_1, true);
motor left_middle(PORT20, gearSetting::ratio6_1, true);
motor left_back(PORT15, gearSetting::ratio6_1, true);
// motor left_raised(PORT8, gearSetting::ratio6_1, false);
motor left_raised(PORT1, gearSetting::ratio6_1, false);


motor right_front(PORT6, gearSetting::ratio6_1, false);
motor right_middle(PORT3, gearSetting::ratio6_1, false);
motor right_back(PORT11, gearSetting::ratio6_1, false);
motor right_raised(PORT5, gearSetting::ratio6_1, true);

motor_group left_motors(left_front, left_middle, left_back, left_raised);
motor_group right_motors(right_front, right_middle, right_back, right_raised);

motor cata_r(PORT4, gearSetting::ratio36_1, false);
motor cata_l(PORT5, gearSetting::ratio36_1, true);

motor_group cata_motors(cata_l, cata_r);

motor intake_combine(PORT8, gearSetting::ratio18_1, false);
motor intake_roller(PORT10, gearSetting::ratio18_1, false);

std::map<std::string, motor &> motor_names = {
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

// ================ SUBSYSTEMS ================
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
inertial imu(PORT10);
gps gps_sensor(PORT11, 0, 0, distanceUnits::in, -90, turnType::left);

// ================ OUTPUTS ================
// Motors
motor left_front(PORT1, true);
motor left_back(PORT2, true);

motor right_front(PORT3);
motor right_back(PORT4);

motor_group left_motors(left_front, left_back);
motor_group right_motors(right_front, right_back);

motor intake_combine(PORT11);
motor intake_roller(PORT12);

std::map<std::string, motor &> motor_names = {
    {"left f", left_front},
    {"left b", left_back},

    {"right f", right_front},
    {"right b", right_back},

};

// ================ SUBSYSTEMS ================

CustomEncoder right_enc = CustomEncoder{Brain.ThreeWirePort.A, 90};
CustomEncoder left_enc = CustomEncoder{Brain.ThreeWirePort.C, 90};

// ======== SUBSYSTEMS ========
PID::pid_config_t drive_pid_cfg =
    {
        .p = .24,
        .i = 0.0,
        .d = .01,
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

OdometryTank odom{left_enc, right_enc, robot_cfg, &imu};
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

#endif

// ================ UTILS ================
std::vector<screen::Page *> pages;



/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{



    pages = {
        new AutoChooser({"Auto 1", "Auto 2", "Auto 3", "Auto 4"}),
        new screen::StatsPage(motor_names),
        new screen::OdometryPage(odom, 12, 12, true),
    };

    screen::start_screen(Brain.Screen, pages, 4);
    imu.calibrate();
    // gps_sensor.calibrate();
}
