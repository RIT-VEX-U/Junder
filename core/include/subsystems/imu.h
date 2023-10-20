#pragma once
#include "vex.h"
#include <atomic>

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
} imu_cfg_t;

class IMU : protected vex::inertial
{
    public:
    IMU(uint32_t port, axisType fwd_axis, axisType vert_axis, bool fwd_is_positive=true);
    IMU(uint32_t port, imu_cfg_t &imu_cfg);
    IMU(uint32_t port);
 
    void orient(axisType fwd_axis, axisType vert_axis, bool fwd_is_positive=true);
    void calibrate(bool blocking=true);
    kinematics_t get_gyro(axisType axis);
    kinematics_t get_accel(axisType axis);

    private:

    friend int calibrate_thread(void* arg);

    inline static constexpr imu_cfg_t default_settings = {
        .fwd_axis=axisType::yaxis,
        .vert_axis=axisType::zaxis,
        .lat_axis=axisType::xaxis,
        .fwd_is_pos=true,
        .pos_mov_avg_buf=0,
        .vel_mov_avg_buf=0,
        .accel_mov_avg_buf=0,
        .min_decay_time_ms=0
    };

    imu_cfg_t* imu_cfg;
    std::atomic<double> acc_cal_thresh;
    std::atomic<bool> is_calibrating;

};