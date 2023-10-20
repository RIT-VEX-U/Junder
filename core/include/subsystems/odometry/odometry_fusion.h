#pragma once
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/utils/moving_average.h"

typedef struct 
{
    const int movingavg_accel_buffersize;
    const int movingavg_vel_buffersize;
    pose_t imu_pose;
    robot_specs_t &robot_cfg;
} odomfus_cfg_t;

class OdometryFusion : OdometryBase
{
public:

    OdometryFusion(vex::motor_group &left_side, vex::motor_group &right_side, odomfus_cfg_t &config, vex::inertial &imu, bool is_async = true);
    OdometryFusion(CustomEncoder &left_custom_enc, CustomEncoder &right_custom_enc, odomfus_cfg_t &config, vex::inertial &imu, bool is_async = true);

    // pose_t update() override;

    // void calibrate();
    // bool is_calibrating();


private:

    pose_t calculate_new_pose(robot_specs_t &config, pose_t &stored_info, double lside_rev, double rside_rev, double side_accel, double angle_deg);

    vex::motor_group *left_side, *right_side;
    CustomEncoder *left_custom_enc, *right_custom_enc;
    odomfus_cfg_t &config;
    vex::inertial &imu;


    // persistent variables
    bool first_start;
    double last_lside_rev, last_rside_rev;
    double cur_vel_x, cur_vel_y, cur_accel_x;
    int last_integral_time_ms;
    vex::timer integral_tmr;
    MovingAverage vel_mav;
    MovingAverage accel_mav;
    
};