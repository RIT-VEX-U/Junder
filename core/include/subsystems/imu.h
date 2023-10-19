#pragma once
#include "vex.h"

typedef struct
{
    double pos;
    double vel;
    double acc;
} kinematics_t;

typedef enum
{
    X, Y, Z
} axis_t;

typedef struct
{
    axis_t fwd_axis;
    bool fwd_is_pos;
    int pos_mov_avg_buf;
    int vel_mov_avg_buf;
    int accel_mov_avg_buf;  
    int min_decay_time_ms;
} imu_cfg_t;

class IMU : vex::inertial
{
    public:
    IMU(uint32_t port, axis_t fwd_axis=Y, bool fwd_is_positive=true);
    IMU(uint32_t port, imu_cfg_t imu_cfg=default_settings);

    void orient(axis_t fwd_axis, bool fwd_is_positive);
    void calibrate();
    kinematics_t get_gyro(axis_t axis);
    kinematics_t get_accel(axis_t axis);

    private:
    
    inline static constexpr imu_cfg_t default_settings = {
        .fwd_axis=Y,
        .fwd_is_pos=true,
        .pos_mov_avg_buf=0,
        .vel_mov_avg_buf=0,
        .accel_mov_avg_buf=0,
        .min_decay_time_ms=0
    };

    double acc_cal_thresh;

};