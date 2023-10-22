#pragma once
#include "vex.h"
#include <tuple>
#include <atomic>
#include <vector>
#include "../core/include/utils/geometry.h"

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
    Mat3 orientation;
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
    void orient(std::vector<std::tuple<vex::axisType, double>> rotation_list);

    void calibrate(bool blocking=true);
    bool is_calibrating();
    kinematics_t get_gyro(rel_axis_t axis);
    kinematics_t get_accel(rel_axis_t axis);

    private:

    friend int calibrate_thread(void* arg);
    friend int integral_callback(void* arg);

    inline static constexpr imu_cfg_t default_settings = {
        .orientation=notransform_matrix,
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

    kinematics_t x_rot, y_rot, z_rot;
    kinematics_t x_lin, y_lin, z_lin;
    
};