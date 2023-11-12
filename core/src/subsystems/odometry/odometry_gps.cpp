#include "../core/include/subsystems/odometry/odometry_gps.h"

OdometryGPS::OdometryGPS(odom_gps_cfg_t &cfg, OdometryTank &enc_odom, gps &gps_sensor, bool is_async) : 
cfg(cfg), enc_odom(enc_odom), OdometryBase(is_async), gps_sensor(gps_sensor)
{

}

OdometryGPS::OdometryGPS(odom_gps_cfg_t &cfg, Odometry3Wheel &enc_odom, gps &gps_sensor, bool is_async) :
cfg(cfg), enc_odom(enc_odom), OdometryBase(is_async), gps_sensor(gps_sensor)
{

}

double OdometryGPS::get_alpha()
{
    
}

pose_t OdometryGPS::update()
{
    pose_t enc_pose = OdometryBase::update();
    pose_t gps_pose = {
        .x = gps_sensor.xPosition(distanceUnits::in),
        .y = gps_sensor.yPosition(distanceUnits::in),
        .rot = gps_sensor.heading(rotationUnits::deg)
    };
    
    // Multiply by the time delta to adjust for runtime inconsistencies
    // (Running faster / slower changes )

    if(fabs(enc_pose.get_point().dist(gps_pose.get_point())) > cfg.cutoff_radius)
    {
        tmr.reset();
        return enc_pose;
    }

    double alpha_adjusted = get_alpha() * tmr.time(sec);
    tmr.reset();

    double x = (alpha_adjusted * gps_pose.x) + ((1-alpha_adjusted) * enc_pose.x);
    double y = (alpha_adjusted * gps_pose.y) + ((1-alpha_adjusted) * enc_pose.y);
    double rot;

    // Handle the issue of averageing when crossing the 360/0 threshold
    if ( fabs(gps_pose.rot - enc_pose.rot) > 180 )
    {
        if (gps_pose.rot < enc_pose.rot)
            gps_pose.rot += 360;
        else
            enc_pose.rot += 360;
    }

    rot = (alpha_adjusted * gps_pose.rot) + ((1-alpha_adjusted) * enc_pose.rot);

    // Wrap the heading between 0 and 360
    if (rot > 360)
        rot -= 360;
    else if (rot < 0)
        rot += 360;

    pose_t out = {
        .x = x,
        .y = y,
        .rot = rot
    };

    enc_odom.set_position(out);
    set_position(out);
}
