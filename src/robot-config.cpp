#include "robot-config.h"
vex::brain Brain;
vex::controller con;

Serializer *serializer = nullptr;

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

vex::motor left_front(vex::PORT1, true);
vex::motor left_back(vex::PORT2, true);

vex::motor right_front(vex::PORT3);
vex::motor right_back(vex::PORT4);

vex::motor_group left_motors(left_front, left_back);
vex::motor_group right_motors(right_front, right_back);

std::map<std::string, vex::motor &> motor_names = {
    {"left f", left_front},
    {"left b", left_back},

    {"right f", right_front},
    {"right b", right_back},

};

OdometryTank odom{left_enc, right_enc, robot_cfg, &imu};
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

// ================ UTILS ================

#endif

PID::pid_config_t cfg = {0, 0, 0, 0, 0.01, PID::ERROR_TYPE::LINEAR};

std::vector<screen::Page *> pages;

screen::SliderWidget p_slider(cfg.p, -0.5, 0.5, Rect{{60, 20}, {210, 60}}, "P");
screen::SliderWidget i_slider(cfg.i, -0.05, 0.05, Rect{{60, 80}, {180, 120}}, "I");
screen::SliderWidget d_slider(cfg.d, -0.05, 0.05, Rect{{60, 140}, {180, 180}}, "D");

screen::ButtonWidget zero_i([]()
                            { cfg.i = 0; },
                            Rect{{180, 80}, {220, 120}}, "0");
screen::ButtonWidget zero_d([]()
                            { cfg.d = 0; },
                            Rect{{180, 140}, {220, 180}}, "0");

GraphDrawer target_graph(Brain.Screen, 40, "", "", vex::color(255, 0, 0), true, -30, 120);
GraphDrawer pos_graph(Brain.Screen, 40, "", "", vex::color(0, 255, 0), false, -30, 120);

auto updatef = [](bool was_pressed, int x, int y)
{
    if (p_slider.update(was_pressed, x, y))
        serializer->set_double("motor_p", cfg.p);

    if (i_slider.update(was_pressed, x, y))
        serializer->set_double("motor_i", cfg.i);

    if (d_slider.update(was_pressed, x, y))
        serializer->set_double("motor_d", cfg.d);

    zero_i.update(was_pressed, x, y);
    zero_d.update(was_pressed, x, y);
};

auto drawf = [](vex::brain::lcd &scr, bool first_draw, unsigned int frame_number)
{
    p_slider.draw(scr, first_draw, frame_number);
    i_slider.draw(scr, first_draw, frame_number);
    d_slider.draw(scr, first_draw, frame_number);
    zero_i.draw(scr, first_draw, frame_number);
    zero_d.draw(scr, first_draw, frame_number);

    target_graph.draw(230, 20, 200, 200);
    pos_graph.draw(230, 20, 200, 200);
};

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    serializer = new Serializer("cata_params.ser");

    cfg.p = serializer->double_or("motor_p", 0.0);
    cfg.i = serializer->double_or("motor_i", 0.0);
    cfg.d = serializer->double_or("motor_d", 0.0);

    fprintf(stderr, "AHHHHHHHHH");

    odom.set_position({72, 62, 0});
    pages = {new AutoChooser({"Auto 1", "Auto 2", "Auto 3", "Auto 4"}), new screen::StatsPage(motor_names), new screen::OdometryPage(odom, 12, 12, true), new screen::FunctionPage(updatef, drawf)};
    imu.calibrate();
    screen::start_screen(Brain.Screen, pages, 3);
}
