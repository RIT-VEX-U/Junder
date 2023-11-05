#include "../core/include/subsystems/imu.h"

IMU::IMU(uint32_t port, axisType xaxis_new, axisType yaxis_new, axisType zaxis_new, bool invert)
: imu_cfg(new imu_cfg_t(default_settings)), vex::inertial(port)
{
    is_calibrating_thresh = false;
    vel_decay_acc_thresh = 0.0;
    orient(xaxis_new, yaxis_new, zaxis_new, invert);

    task(integral_thread, this);
}

IMU::IMU(uint32_t port, imu_cfg_t &imu_cfg)
: imu_cfg(&imu_cfg), vex::inertial(port)
{
    is_calibrating_thresh = false;
    vel_decay_acc_thresh = 0.0;
    task(integral_thread, this);
}

IMU::IMU(uint32_t port) 
: imu_cfg(new imu_cfg_t(default_settings)), vex::inertial(port) 
{
    is_calibrating_thresh = false;
    vel_decay_acc_thresh = 0.0;
    task(integral_thread, this);
}

double is_equal(axisType a1, axisType a2){
    if (a1 == a2)
        return 1.0;
    else
        return 0.0;
}

void IMU::orient(axisType xaxis_new, axisType yaxis_new, axisType zaxis_new, bool invert)
{
    if(xaxis_new == yaxis_new
    || yaxis_new == zaxis_new
    || zaxis_new == xaxis_new)
    {
        printf("IMU::orient - Axes cannot be the same!, \
                using default orientation instead.\n");
        return;
    }

    Mat3 new_orient;

    // Row 1
    new_orient.X11 = is_equal(xaxis_new, xaxis);
    new_orient.X12 = is_equal(xaxis_new, yaxis);
    new_orient.X13 = is_equal(xaxis_new, zaxis);

    // Row 2
    new_orient.X21 = is_equal(yaxis_new, xaxis);
    new_orient.X22 = is_equal(yaxis_new, yaxis);
    new_orient.X23 = is_equal(yaxis_new, zaxis);

    // Row 3
    new_orient.X31 = is_equal(zaxis_new, xaxis);
    new_orient.X32 = is_equal(zaxis_new, yaxis);
    new_orient.X33 = is_equal(zaxis_new, zaxis);

    if(invert)
        new_orient = new_orient * -1;

    imu_cfg->orientation = new_orient;    
}

void IMU::orient(std::vector<std::tuple<vex::axisType, double>> rotation_list)
{
    Mat3 out = identity_matrix;

    for(int i = 0; i < rotation_list.size(); i++)
        out = out * get_rotation_matrix(
            std::get<0>(rotation_list[i]), 
            std::get<1>(rotation_list[i]));
    
    imu_cfg->orientation = out;
}

int calibrate_thread(void* arg)
{
    IMU &imu = *(IMU*)arg;
    vex::timer tmr;
    double max_acc = 0;

    std::vector<double> x_acc_val, y_acc_val, z_acc_val;

    // For 3 seconds, get the max baseline acceleration at standstill
    while(tmr.time() < 3000)
    {

        point3_t acc_raw = {
            .x = imu.acceleration(xaxis),
            .y = imu.acceleration(yaxis), 
            .z = imu.acceleration(zaxis)
        };

        x_acc_val.push_back(acc_raw.x);
        y_acc_val.push_back(acc_raw.y);
        z_acc_val.push_back(acc_raw.z);

        // Reoriented current IMU data
        point3_t acc = acc_raw * imu.imu_cfg->orientation;

        // Gain data for velocity decay threshold
        if(fabs(acc.x) > max_acc)
            max_acc = fabs(acc.x);

        if(fabs(acc.y) > max_acc) 
            max_acc = fabs(acc.y);

        vexDelay(10);
    }

    double x_mean = mean(x_acc_val);
    double y_mean = mean(y_acc_val);
    double z_mean = mean(z_acc_val);

    // Find raw offset values (mean)
    imu.accel_offset.x = x_mean;
    imu.accel_offset.y = y_mean;
    imu.accel_offset.z = z_mean;

    // Find raw threshold values (3*std_dev, filters 99.7% of standstill noise)
    double accel_stddev_x = sqrt(variance(x_acc_val, x_mean));
    double accel_stddev_y = sqrt(variance(y_acc_val, y_mean));
    double accel_stddev_z = sqrt(variance(z_acc_val, z_mean));

    imu.accel_thresh.x = 3 * accel_stddev_x;
    imu.accel_thresh.y = 3 * accel_stddev_y;
    imu.accel_thresh.z = 3 * accel_stddev_z;

    printf("zavg: %f, zvar: %f, zoff: %f, zthr: %f \n", z_mean, variance(z_acc_val, z_mean), imu.accel_offset.z, imu.accel_thresh.z);

    // TODO
    // Refine the orientation rotation matrix by measuring the vector angle of gravity

    // The threshold for decaying velocity will be when acceleration is
    // below twice what we measured here
    imu.vel_decay_acc_thresh = imu.imu_cfg->decay_threshold_factor * max_acc;
    printf("decay: %f\n", (double)imu.vel_decay_acc_thresh);

    // Signal that calibrating is finished
    imu.is_calibrating_thresh = false;
    
    return 0;
}

vex::task *cal_task = NULL;

// TODO separate is_calibrating and calibrate_thread_done, create is_calibrating overload function
void IMU::calibrate(bool blocking)
{
    vex::inertial::calibrate();
    
    if(!is_calibrating_thresh)
    {   
        if(cal_task != NULL)
            delete cal_task;

        // Kick off a task to:
        // calibrate for a minimum threshold (for velocity decay)
        // runs while inertial::calibrate is running
        cal_task = new vex::task(calibrate_thread, this);
        is_calibrating_thresh = true;
    }

    if(blocking)
    {
        // Wait until vex calibration AND this calibration is finished
        while(vex::inertial::isCalibrating() || is_calibrating_thresh) { vexDelay(10); }
    }
}

bool IMU::is_calibrating()
{
    return vex::inertial::isCalibrating() || is_calibrating_thresh;
}

int integral_thread(void* arg)
{
    IMU &imu = *(IMU*)arg;

    imu.x_acc_movavg = new MovingAverage(imu.imu_cfg->accel_mov_avg_buf);
    imu.y_acc_movavg = new MovingAverage(imu.imu_cfg->accel_mov_avg_buf);
    imu.z_acc_movavg = new MovingAverage(imu.imu_cfg->accel_mov_avg_buf);

    imu.x_vel_movavg = new MovingAverage(imu.imu_cfg->vel_mov_avg_buf);
    imu.y_vel_movavg = new MovingAverage(imu.imu_cfg->vel_mov_avg_buf);
    imu.z_vel_movavg = new MovingAverage(imu.imu_cfg->vel_mov_avg_buf);

    imu.x_pos_movavg = new MovingAverage(imu.imu_cfg->pos_mov_avg_buf);
    imu.y_pos_movavg = new MovingAverage(imu.imu_cfg->pos_mov_avg_buf);
    imu.z_pos_movavg = new MovingAverage(imu.imu_cfg->pos_mov_avg_buf);

    while(true)
    {
        while(imu.is_calibrating()) {vexDelay(100);}

        double time_delta_sec = imu.integral_tmr.time() / 1000.0;
        imu.integral_tmr.reset();

        kinematics_t imu_xlin_loc = imu.x_lin;
        kinematics_t imu_ylin_loc = imu.y_lin;
        kinematics_t imu_zlin_loc = imu.z_lin;

        // Get raw accel values, apply offset & threshold
        double x_acc_raw = imu.acceleration(xaxis) - imu.accel_offset.x;
        double y_acc_raw = imu.acceleration(yaxis) - imu.accel_offset.y;
        double z_acc_raw = imu.acceleration(zaxis) - imu.accel_offset.z;

        if(fabs(x_acc_raw) < imu.accel_thresh.x)
            x_acc_raw = 0;
        if(fabs(y_acc_raw) < imu.accel_thresh.y)
            y_acc_raw = 0;
        if(fabs(z_acc_raw) < imu.accel_thresh.z)
            z_acc_raw = 0;

        // Convert from Gs to in/sec^2
        imu.x_acc_movavg->add_entry(x_acc_raw * 386.1 ); 
        imu.y_acc_movavg->add_entry(y_acc_raw * 386.1 );
        imu.z_acc_movavg->add_entry(z_acc_raw * 386.1 );

        // Calculate Velocity

        double x_vel = imu_xlin_loc.vel + (imu.x_acc_movavg->get_average() * time_delta_sec);
        double y_vel = imu_ylin_loc.vel + (imu.y_acc_movavg->get_average() * time_delta_sec);
        double z_vel = imu_zlin_loc.vel + (imu.z_acc_movavg->get_average() * time_delta_sec);

        // Hacky method for resetting velocity when not moving
        // if (fabs(x_acc_raw) < imu.vel_decay_acc_thresh)
        //     x_vel = 0;
        // if (fabs(y_acc_raw) < imu.vel_decay_acc_thresh)
        //     y_vel = 0;
        // if (fabs(z_acc_raw) < imu.vel_decay_acc_thresh)
        //     z_vel = 0;

        imu.x_vel_movavg->add_entry(x_vel);
        imu.y_vel_movavg->add_entry(y_vel);
        imu.z_vel_movavg->add_entry(z_vel);

        // Calculate Position

        double x_pos = imu_xlin_loc.pos + (imu.x_vel_movavg->get_average() * time_delta_sec);
        double y_pos = imu_ylin_loc.pos + (imu.y_vel_movavg->get_average() * time_delta_sec);
        double z_pos = imu_zlin_loc.pos + (imu.z_vel_movavg->get_average() * time_delta_sec);

        imu.x_pos_movavg->add_entry(x_pos);
        imu.y_pos_movavg->add_entry(y_pos);
        imu.z_pos_movavg->add_entry(z_pos);

        // Return values

        imu.mux.lock();
        imu.x_lin.acc = imu.x_acc_movavg->get_average();
        imu.y_lin.acc = imu.y_acc_movavg->get_average();
        imu.z_lin.acc = imu.z_acc_movavg->get_average();

        imu.x_lin.vel = imu.x_vel_movavg->get_average();
        imu.y_lin.vel = imu.y_vel_movavg->get_average();
        imu.z_lin.vel = imu.z_vel_movavg->get_average();

        imu.x_lin.pos = imu.x_pos_movavg->get_average();
        imu.y_lin.pos = imu.y_pos_movavg->get_average();
        imu.z_lin.pos = imu.z_pos_movavg->get_average();
        imu.mux.unlock();

        vexDelay(INT_PERIOD_MS);
    }

    return 0;
}

kinematics_t IMU::get_gyro(axisType axis)
{
    return kinematics_t{};
}

kinematics_t IMU::get_accel(axisType axis)
{
    point3_t acc, vel, pos;

    mux.lock();
    acc = {
        .x = x_lin.acc,
        .y = y_lin.acc,
        .z = z_lin.acc
    };

    vel = {
        .x = x_lin.vel,
        .y = y_lin.vel,
        .z = z_lin.vel
    };

    pos = {
        .x = x_lin.pos,
        .y = y_lin.pos,
        .z = z_lin.pos
    };
    mux.unlock();

    point3_t reorient_acc = imu_cfg->orientation * acc;
    point3_t reorient_vel = imu_cfg->orientation * vel;
    point3_t reorient_pos = imu_cfg->orientation * pos;

    kinematics_t out;

    if (axis == xaxis)
        out = kinematics_t {
            .acc = reorient_acc.x,
            .vel = reorient_vel.x,
            .pos = reorient_pos.x
        };
    else if(axis == yaxis)
        out = kinematics_t {
            .acc = reorient_acc.y,
            .vel = reorient_vel.y,
            .pos = reorient_pos.y
        };
    else if(axis == zaxis)
        out = kinematics_t {
            .acc = reorient_acc.z,
            .vel = reorient_vel.z,
            .pos = reorient_pos.z
        };

    return out;
}