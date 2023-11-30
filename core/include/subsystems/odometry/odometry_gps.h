#pragma once
#include <vector>
#include <tuple>
#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/subsystems/odometry/odometry_3wheel.h"
#include "../core/include/utils/vector2d.h"

typedef struct
{
    double cutoff_radius;
    double alpha_scalar;

} odom_gps_cfg_t;

class OdometryGPS : public OdometryBase
{
    public:

    OdometryGPS(odom_gps_cfg_t &cfg, OdometryTank &enc_odom, gps &gps_sensor, bool is_async=true);
    OdometryGPS(odom_gps_cfg_t &cfg, Odometry3Wheel &enc_odom, gps &gps_sensor, bool is_async=true);

    /**
     * Update the current position on the field based on the sensors
     * @return the location that the robot is at after the odometry does its calculations
     */
    pose_t update() override;

    double get_alpha();

    private:

    odom_gps_cfg_t &cfg;
    OdometryBase &enc_odom;
    gps &gps_sensor;
    timer tmr;
};