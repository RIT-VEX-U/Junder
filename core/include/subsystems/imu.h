#pragma once
#include "vex.h"
#include <atomic>

#define INT_PERIOD_MS 10

using namespace vex;

typedef struct
{
    double pos;
    double vel;
    double acc;
} kinematics_t;

typedef struct
{
    axisType fwd_axis;
    axisType vert_axis;
    axisType lat_axis;
    bool fwd_is_pos;
    int pos_mov_avg_buf;
    int vel_mov_avg_buf;
    int accel_mov_avg_buf;  
    int min_decay_time_ms;
    int decay_threshold_factor;
} imu_cfg_t;

typedef enum
{
    imu_x, imu_y, imu_z, 
    robot_x, robot_y, robot_z
} rel_axis_t;

class IMU : protected vex::inertial
{
    public:
    IMU(uint32_t port, axisType fwd_axis, axisType vert_axis, bool fwd_is_positive=true);
    IMU(uint32_t port, imu_cfg_t &imu_cfg);
    IMU(uint32_t port);
 
    void orient(axisType fwd_axis, axisType vert_axis, bool fwd_is_positive=true);
    void calibrate(bool blocking=true);
    bool is_calibrating();
    kinematics_t get_gyro(rel_axis_t axis);
    kinematics_t get_accel(rel_axis_t axis);

    private:

    friend int calibrate_thread(void* arg);
    friend int integral_callback(void* arg);

    inline static constexpr imu_cfg_t default_settings = {
        .fwd_axis=axisType::yaxis,
        .vert_axis=axisType::zaxis,
        .lat_axis=axisType::xaxis,
        .fwd_is_pos=true,
        .pos_mov_avg_buf=0,
        .vel_mov_avg_buf=0,
        .accel_mov_avg_buf=0,
        .min_decay_time_ms=0,
        .decay_threshold_factor=2
    };

    imu_cfg_t* imu_cfg;
    std::atomic<double> acc_cal_thresh;
    std::atomic<bool> is_calibrating_thresh;
    vex::timer integral_tmr;

    kinematics_t x_imu_rot, y_imu_rot, z_imu_rot;
    kinematics_t x_imu_lin, y_imu_lin, z_imu_lin;
    kinematics_t x_robot_rot, y_robot_rot, z_robot_rot;
    kinematics_t x_robot_lin, y_robot_lin, z_robot_lin;
    
};