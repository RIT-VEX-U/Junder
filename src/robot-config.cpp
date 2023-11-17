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

// PID::pid_config_t pcfg = {.p = 0.001, 0, 0, 0, 0, PID::LINEAR};
FeedForward::ff_config_t ffcfg = {.kS = 0.0, .kV = 0.0007, .kA = 0, .kG = 0};

FeedForward ff = FeedForward(ffcfg);
TakeBackHalf bb = TakeBackHalf(0.00008,0.75, 10.0);

MovingAverage avger(10);

screen::SliderWidget tbh(bb.TBH_gain, 0.0, 0.0005, Rect{{60, 40}, {380, 80}}, "TBH Gain");
screen::SliderWidget split(bb.first_cross_split, 0.0, 1.0, Rect{{60, 90}, {380, 130}}, "first cross split");

auto update = [](bool wp, int x, int y)
{
    tbh.update(wp, x, y);
    split.update(wp, x, y);
};
auto draw = [](brain::lcd &scr, bool f, int n)
{
    tbh.draw(scr, f, n);
    split.draw(scr, f, n);
};

double combine_testing_volt = 12, roller_testing_volt = 12;

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    bb.set_limits(0.0, 1.0);

    static Rect com_slider_rect = {
        .min = {60, 80}, // X=60, Y=80
        .max = {60 + 200, 80 + 30} // Width=200, Height=30
    };
    static screen::SliderWidget combine_slider(combine_testing_volt, 0.0, 12.0, com_slider_rect, "Combine Voltage");

    static Rect rol_slider_rect = {
        .min = {com_slider_rect.min.x, com_slider_rect.max.y + 20},
        .max = {com_slider_rect.min.x + 200, com_slider_rect.max.y + 50}
    };
    static screen::SliderWidget roller_slider(roller_testing_volt, 0.0, 12.0, rol_slider_rect, "Roller Voltage");

    static screen::ButtonWidget combine_rev_btn([&](){
        static bool is_com_rev = true;
        intake_combine.setReversed(is_com_rev = !is_com_rev);
        }, {
            {com_slider_rect.max.x + 20, com_slider_rect.min.y}, 
            {com_slider_rect.max.x + 20 + 40, com_slider_rect.min.y + 40}
        }, "Rev");

    static screen::ButtonWidget roller_rev_btn([&](){
        static bool is_rol_rev = true;
        intake_roller.setReversed(is_rol_rev = !is_rol_rev);
        }, {
            {rol_slider_rect.max.x + 20, rol_slider_rect.min.y}, 
            {rol_slider_rect.max.x + 20 + 40, rol_slider_rect.min.y + 40}
        }, "Rev");

    pages = {
        new AutoChooser({"Auto 1", "Auto 2", "Auto 3", "Auto 4"}),
        new screen::StatsPage(motor_names),
        new screen::OdometryPage(odom, 12, 12, true),
        new screen::FunctionPage(update, draw),
        new screen::FunctionPage([&](bool wp, int x, int y){
            combine_slider.update(wp, x, y);
            roller_slider.update(wp, x, y);
            combine_rev_btn.update(wp, x, y);
            roller_rev_btn.update(wp, x, y);
        },
        [&](brain::lcd lcd, bool f, int n){
            combine_slider.draw(lcd, f, n);
            roller_slider.draw(lcd, f, n);
            combine_rev_btn.draw(lcd, f, n);
            roller_rev_btn.draw(lcd, f, n);
        }), 
    };

    screen::start_screen(Brain.Screen, pages, 2);
    imu.calibrate();
    // gps_sensor.calibrate();
}
