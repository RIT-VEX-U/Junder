#include "../core/include/subsystems/odometry/odometry_fusion.h"
#include "../core/include/utils/vector2d.h"
#include "../core/include/utils/moving_average.h"



OdometryFusion::OdometryFusion(vex::motor_group &left_side, vex::motor_group &right_side, odomfus_cfg_t &config, vex::inertial &imu, bool is_async)
: left_side(&left_side), right_side(&right_side), config(config), imu(imu),vel_mav(config.movingavg_vel_buffersize), accel_mav(config.movingavg_accel_buffersize), OdometryBase(is_async)
{

}

OdometryFusion::OdometryFusion(CustomEncoder &left_custom_enc, CustomEncoder &right_custom_enc, odomfus_cfg_t &config, vex::inertial &imu, bool is_async)
: left_custom_enc(&left_custom_enc), right_custom_enc(&right_custom_enc), config(config), imu(imu), vel_mav(config.movingavg_vel_buffersize), accel_mav(config.movingavg_accel_buffersize), OdometryBase(is_async)
{

}

/**
 * Get information from the input hardware and an existing position, and calculate a new current position
 */
pose_t OdometryFusion::calculate_new_pose(robot_specs_t &config, pose_t &stored_info, double lside_rev, double rside_rev, double side_accel, double angle_deg)
{
    if(first_start)
    {
        last_integral_time_ms = integral_tmr.time();
        last_lside_rev = lside_rev;
        last_rside_rev = rside_rev;
    }
    
    double lside_delta_rev = lside_rev - last_lside_rev;
    double rside_delta_rev = rside_rev - last_rside_rev;

    double avg_delta_rev = (lside_delta_rev + rside_delta_rev) / 2.0;
    double avg_delta_dist = avg_delta_rev * PI * config.odom_wheel_diam * config.odom_gear_ratio;
    
    // Perform a double integral on the acceleration
    double delta_time_ms = integral_tmr.time();
    integral_tmr.reset();

    
    
    
    // Store persistent variables for future use
    
    last_lside_rev = lside_rev;
    last_rside_rev = rside_rev;
}