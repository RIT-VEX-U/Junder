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
    pose_t robot_pose = get_position();

    // Part 1 - relative angle to the center of the field (as a scalar)
    Vector2D position_vec({.x=72-robot_pose.x, .y=72-robot_pose.y});
    position_vec = position_vec.normalize();
    Vector2D rotation_vec(deg2rad(robot_pose.rot), 1);
    double dot_prod = position_vec.dot(rotation_vec);

    // Part 2 - Distance to center of field
    double dist_scalar = robot_pose.get_point().dist({72, 72}) / 101.8; // (0 to 1)
    
    double alpha = ((dist_scalar * dot_prod) + 1) / 2.0;

    return alpha;
}

pose_t OdometryGPS::update()
{
    pose_t enc_pose = enc_odom.update();
    pose_t gps_pose = {
        .x = gps_sensor.xPosition(distanceUnits::in) + 72,
        .y = gps_sensor.yPosition(distanceUnits::in) + 72,
        .rot = gps_sensor.heading(rotationUnits::deg)
    };
    
    // Multiply by the time delta to adjust for runtime inconsistencies
    // (Running faster / slower changes )

    if(fabs(enc_pose.get_point().dist(gps_pose.get_point())) > cfg.cutoff_radius)
    {
        tmr.reset();
        return enc_pose;
    }

    double alpha_adjusted = get_alpha() * tmr.time(sec) 
        * cfg.alpha_scalar * gps_sensor.quality() / 100.0;
    tmr.reset();

    double x, y, rot;

    // Check if the point is within a reasonable distance to the encoder odometry.
    // If not, ignore the data since it's probably bad.
    if (enc_pose.get_point().dist(gps_pose.get_point()) < cfg.cutoff_radius)
    {
        x = (alpha_adjusted * gps_pose.x) + ((1-alpha_adjusted) * enc_pose.x);
        y = (alpha_adjusted * gps_pose.y) + ((1-alpha_adjusted) * enc_pose.y);

        // Handle the issue of averageing when crossing the 360/0 threshold
        if ( fabs(gps_pose.rot - enc_pose.rot) > 180 )
        {
            if (gps_pose.rot < enc_pose.rot)
                gps_pose.rot += 360;
            else
                enc_pose.rot += 360;
        }

        rot = (alpha_adjusted * gps_pose.rot) + ((1-alpha_adjusted) * enc_pose.rot);
        rot = fmod(rot, 360.0);
        if(rot < 0)
            rot += 360;

    } else
    {
        x = enc_pose.x;
        y = enc_pose.y;
        rot = enc_pose.rot;
    }

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

    return out;
}
