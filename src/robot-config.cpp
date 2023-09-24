#include "robot-config.h"

vex::brain Brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors
CustomEncoder left_odom_wheel = CustomEncoder(Brain.ThreeWirePort.A, 90);
CustomEncoder right_odom_wheel = CustomEncoder(Brain.ThreeWirePort.C, 90);

// Analog sensors
inertial imu(PORT10);

// ================ OUTPUTS ================
// Motors
motor left_front = motor(PORT1, true);
motor left_back = motor(PORT2, true);
motor right_front = motor(PORT3);
motor right_back = motor(PORT4);
motor_group left_drive = motor_group(left_front, left_back);
motor_group right_drive = motor_group(right_front, right_back);

// ================ SUBSYSTEMS ================
PID::pid_config_t drive_pid_cfg =
{
  .p = .1,
  .i = .001,
  .d = .008,
  .deadband = 0.3,
  .on_target_time = 0
};

PID::pid_config_t turn_pid_cfg = 
{
  .p = 0.025,
  .i = 0.01,
  .d = 0.0015,
  .deadband = 5,
  .on_target_time = 0.1
};

FeedForward::ff_config_t drive_ff_cfg = 
{
  .kS = .05,
  .kV = .014552
};

FeedForward::ff_config_t turn_ff_cfg = 
{

};

MotionController::m_profile_cfg_t drive_profile_cfg = 
{
  .max_v = 20,
  .accel = 20,
  .ff_cfg = drive_ff_cfg,
  .pid_cfg = drive_pid_cfg
};

MotionController::m_profile_cfg_t turn_profile_cfg = 
{

};


robot_specs_t robot_cfg = {
  .robot_radius = 12, // inches
  .odom_wheel_diam = 2.84, // inches
  .odom_gear_ratio = 1.03, // inches
  .dist_between_wheels = 9.18, // inches
  .drive_correction_cutoff = 12, //inches
  .drive_feedback = new PIDFF(drive_pid_cfg, drive_ff_cfg),//new MotionController(drive_profile_cfg),
  .turn_feedback = new PIDFF(turn_pid_cfg, turn_ff_cfg),//new MotionController(turn_profile_cfg),
  .correction_pid = (PID::pid_config_t)
  {
    .p = .01,
  }
};

OdometryTank odometry(left_odom_wheel, right_odom_wheel, robot_cfg);
TankDrive drive_system(left_drive, right_drive, robot_cfg, &odometry);


// ================ UTILS ================


/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
*/
void robot_init()
{

}
