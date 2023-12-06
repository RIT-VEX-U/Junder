#pragma once
#include "../core/include/subsystems/odometry/odometry_3wheel.h"
#include "../core/include/subsystems/odometry/odometry_tank.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/utils/vector2d.h"
#include <tuple>
#include <vector>

typedef struct {
  double cutoff_radius;
  /**
   * normalized distance (0 at center 1 at corner) raised to this power.
   * Higher values mean stay good for farther then switch rapidly at the end
   * Lower values mean only prefer the gps when you're far from the walls
   */
  double distance_power;
  /**
   * Controls how much we want to discard old updates.
   * If time factor is 0, we don't care how long ago the update came in, we're
   * still going to use the gps] If time factor is high, we will prefer odometry
   * if the gps update came in a while ago The actual value of this is
   * unbeknownst to me.
   */
  double time_factor;

} odom_gps_cfg_t;

class OdometryGPS : OdometryBase {
public:
  OdometryGPS(odom_gps_cfg_t &cfg, OdometryTank &enc_odom, gps &gps_sensor,
              bool is_async = true);
  OdometryGPS(odom_gps_cfg_t &cfg, Odometry3Wheel &enc_odom, gps &gps_sensor,
              bool is_async = true);

  /**
   * Update the current position on the field based on the sensors
   * @return the location that the robot is at after the odometry does its
   * calculations
   */
  pose_t update() override;

private:
  /**
   * @return alpha: 0 when we should favor GPS position. 1 when we should favor
   * encoder based odom. 0 when we're far away from the wall we're looking at
   * and gps is worse. 1 when we're up close to the wall we're looking at and
   * gps is better
   */
  double get_alpha();

  odom_gps_cfg_t &cfg;
  OdometryBase &enc_odom;
  gps &gps_sensor;
  timer tmr;
};