#include "../core/include/subsystems/imu.h"
#include <utility>

IMU::IMU(uint32_t port, axisType fwd_axis, axisType lat_axis, bool fwd_is_positive)
: imu_cfg(new imu_cfg_t(default_settings)), vex::inertial(port)
{
    is_calibrating = false;
    acc_cal_thresh = 0.0;
    orient(fwd_axis, lat_axis, fwd_is_positive);
}

IMU::IMU(uint32_t port, imu_cfg_t &imu_cfg)
: imu_cfg(&imu_cfg), vex::inertial(port)
{
    is_calibrating = false;
    acc_cal_thresh = 0.0;
}

IMU::IMU(uint32_t port) 
: imu_cfg(new imu_cfg_t(default_settings)), vex::inertial(port) 
{
    is_calibrating = false;
    acc_cal_thresh = 0.0;
}

void IMU::orient(axisType fwd_axis, axisType lat_axis, bool fwd_is_positive)
{
    if(fwd_axis == lat_axis)
    {
        printf("IMU::orient - fwd_axis and vert_axis cannot be the same!, \
                using default orientation instead.\n");
        return;
    }

    imu_cfg->fwd_axis=fwd_axis;
    imu_cfg->lat_axis=lat_axis;
    imu_cfg->fwd_is_pos=fwd_is_positive;

    // Given two different axes, find the third and assign it
    switch(fwd_axis)
    {
        case axisType::xaxis:
            if(lat_axis == axisType::yaxis)
                imu_cfg->vert_axis = axisType::zaxis;
            else if(lat_axis == axisType::zaxis)
                imu_cfg->vert_axis = axisType::yaxis;
        break;
        case axisType::yaxis:
            if(lat_axis == axisType::xaxis)
                imu_cfg->vert_axis = axisType::zaxis;
            else if(lat_axis == axisType::zaxis)
                imu_cfg->vert_axis = axisType::xaxis;
        break;
        case axisType::zaxis:
            if(lat_axis == axisType::yaxis)
                imu_cfg->vert_axis = axisType::xaxis;
            else if(lat_axis == axisType::xaxis)
                imu_cfg->vert_axis = axisType::yaxis;
    }
    
}

int calibrate_thread(void* arg)
{
    IMU &imu = *(IMU*)arg;
    vex::timer tmr;
    double max_acc = 0;
    axisType fwd_axis = imu.imu_cfg->fwd_axis;
    axisType lat_axis = imu.imu_cfg->lat_axis;
    
    // For 3 seconds, get the max baseline acceleration at standstill
    while(tmr.time() < 3000)
    {
        double acc1 = imu.acceleration(fwd_axis);
        double acc2 = imu.acceleration(lat_axis);

        if(fabs(acc1) > max_acc)
            max_acc = fabs(acc1);

        if(fabs(acc2) > max_acc) 
            max_acc = fabs(acc2);

        vexDelay(10);
    }

    // The threshold for decaying velocity will be when acceleration is
    // below twice what we measured here
    imu.acc_cal_thresh = 2 * max_acc;

    // Signal that calibrating is finished
    imu.is_calibrating = false;
    
    return 0;
}

vex::task *cal_task = NULL;

// TODO separate is_calibrating and calibrate_thread_done, create is_calibrating overload function
void IMU::calibrate(bool blocking)
{
    vex::inertial::calibrate();
    
    if(!is_calibrating)
    {   
        if(cal_task != NULL)
            delete cal_task;

        // Kick off a task to:
        // calibrate for a minimum threshold (for velocity decay)
        // runs while inertial::calibrate is running
        cal_task = new vex::task(calibrate_thread, this);
        is_calibrating = true;
    }

    if(blocking)
    {
        // Wait until vex calibration AND this calibration is finished
        while(vex::inertial::isCalibrating() || is_calibrating) { vexDelay(10); }
    }
}

kinematics_t IMU::get_gyro(axisType axis)
{
    return kinematics_t{};
}

kinematics_t IMU::get_accel(axisType axis)
{
    return kinematics_t{};
}