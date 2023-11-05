#pragma once
#include "vex.h"
#include <tuple>
#include <atomic>
#include <vector>
#include "../core/include/utils/geometry.h"
#include "../core/include/utils/moving_average.h"

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

class IMU : protected vex::inertial
{
    public:
    IMU(uint32_t port, axisType xaxis_new, axisType yaxis_new, axisType zaxis_new, bool invert);
    IMU(uint32_t port, imu_cfg_t &imu_cfg);
    IMU(uint32_t port);
 
    void orient(axisType xaxis_new, axisType yaxis_new, axisType zaxis_new, bool invert);
    void orient(std::vector<std::tuple<vex::axisType, double>> rotation_list);

    void calibrate(bool blocking=true);
    bool is_calibrating();
    kinematics_t get_gyro(axisType axis);
    kinematics_t get_accel(axisType axis);

    private:

    friend int calibrate_thread(void* arg);
    friend int integral_thread(void* arg);

    inline static constexpr imu_cfg_t default_settings = {
        .orientation=identity_matrix,
        .pos_mov_avg_buf=0,
        .vel_mov_avg_buf=0,
        .accel_mov_avg_buf=0,
        .min_decay_time_ms=0,
        .decay_threshold_factor=2
    };

    imu_cfg_t* imu_cfg;
    std::atomic<double> vel_decay_acc_thresh;
    std::atomic<bool> is_calibrating_thresh;
    vex::timer integral_tmr;
    vex::mutex mux;

    kinematics_t x_rot, y_rot, z_rot;
    kinematics_t x_lin, y_lin, z_lin;

    MovingAverage *x_acc_movavg, *y_acc_movavg, *z_acc_movavg;
    MovingAverage *x_vel_movavg, *y_vel_movavg, *z_vel_movavg;
    MovingAverage *x_pos_movavg, *y_pos_movavg, *z_pos_movavg;

    point3_t accel_offset;
    point3_t accel_thresh;
};

int calibrate_thread(void* arg);
int integral_thread(void* arg);