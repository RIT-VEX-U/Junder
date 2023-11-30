#include "robot-config.h"
brain Brain;
controller con;

using namespace vex;

#ifdef COMP_BOT

// ================ INPUTS ================
// Digital sensors

// Analog sensors
inertial imu(PORT8);

// ================ OUTPUTS ================
// Motors
constexpr gearSetting drive_gears = gearSetting::ratio18_1;
motor left_front_front(PORT17, drive_gears, true); // Final Port
motor left_front_back(PORT18, drive_gears, false); // Final Port
motor left_back_front(PORT19, drive_gears, false); // Final Port
motor left_back_back(PORT20, drive_gears, true); // Final Port

motor right_front_front(PORT14, drive_gears, false); // Final Port
motor right_front_back(PORT13, drive_gears, true); // Final Port
motor right_back_front(PORT12, drive_gears, true); // Final Port
motor right_back_back(PORT11, drive_gears, false); // Final Port

motor_group left_motors(left_front_front, left_front_back, left_back_front, left_back_back);
motor_group right_motors(right_front_front, right_front_back, right_back_front, right_back_back);

motor cata_r(PORT2, gearSetting::ratio36_1, false); // Final Port
motor cata_l(PORT10, gearSetting::ratio36_1, true); // Final Port

motor_group cata_motors(cata_l, cata_r);

motor intake_combine(PORT1, gearSetting::ratio18_1, false); // Final Port
motor intake_roller(PORT9, gearSetting::ratio18_1, false); // Final Port

motor_group intake_motors = {intake_combine, intake_roller};

std::map<std::string, motor &> motor_names = {
    {"left ff", left_front_front},
    {"left fb", left_front_back},
    {"left bf", left_back_front},
    {"left bb", left_back_back},

    {"right ff", right_front_front},
    {"right fb", right_front_back},
    {"right bf", right_back_front},
    {"right bb", right_back_back},

    {"cata L", cata_l},
    {"cata R", cata_r},

    {"intake H", intake_combine},
    {"intake L", intake_roller},

};

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg =
    {
        .p = .2,
        .i = 0.0,
        .d = .021,
        .deadband = 0.1,
        .on_target_time = 0};

PID::pid_config_t turn_pid_cfg =
    {
        .p = 0.03,
        .i = 0.00,
        .d = 0.0020,
        .deadband = 1,
        .on_target_time = 0.1};


MotionController::m_profile_cfg_t drive_mc_cfg{
    .max_v= 72.50,
    .accel = 190.225,
    .pid_cfg = drive_pid_cfg,
    .ff_cfg = FeedForward::ff_config_t{
        .kA = 0.0029,
        .kV = 0.0131,
        .kS = 0.05,
        .kG = 0,
    }
};
MotionController drive_mc{drive_mc_cfg};

robot_specs_t robot_cfg = {
    .robot_radius = 12,            // inches
    .odom_wheel_diam = 3.15,          // inches
    .odom_gear_ratio = .5,        
    .dist_between_wheels = 10.6,    // inches
    .drive_correction_cutoff = 12, // inches
    .drive_feedback = new PID(drive_pid_cfg),//&drive_mc,
    .turn_feedback = new PID(turn_pid_cfg),
    .correction_pid = (PID::pid_config_t){
        .p = .03,
    }};

OdometryTank odom{left_motors, right_motors, robot_cfg,&imu};
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

vex::optical intake_watcher(vex::PORT4);
vex::optical cata_watcher(vex::PORT15); // Final Port
vex::pot cata_pot(Brain.ThreeWirePort.E);

// VISION PORT 16 Final Port

vex::digital_out left_wing(Brain.ThreeWirePort.G);
vex::digital_out right_wing(Brain.ThreeWirePort.H);

CataSys cata_sys(intake_watcher, cata_pot, cata_watcher, cata_motors, intake_roller, intake_combine);

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
        cata_sys.Page(),
    };

    screen::start_screen(Brain.Screen, pages, 2);
    // imu.calibrate();
    // gps_sensor.calibrate();
}
