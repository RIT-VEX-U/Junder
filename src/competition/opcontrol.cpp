#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"



/**
 * Main entrypoint for the driver control period
 */
void opcontrol()
{

    con.ButtonUp.pressed([]()
                         { fw.spin_rpm(1000); });
    con.ButtonLeft.pressed([]()
                           { fw.spin_rpm(500); });
    con.ButtonRight.pressed([]()
                            { fw.spin_rpm(750); });
    con.ButtonDown.pressed([]()
                           { fw.spin_rpm(0); });

    odom_gps.set_position({.x=24, .y=24, .rot=90});

    while (imu.isCalibrating() || gps_sensor.isCalibrating())
    {
        vexDelay(20);
    }

    static int x_tile = 1;
    static int y_tile = 1;
    static int rot_index = 0;

    static const double tile_size = 24.0;

    con.ButtonA.pressed([](){
        // Print expected
        printf("%f, %f, %f, ",
        x_tile * tile_size, 
        y_tile * tile_size, 
        rot_index * 90.0);
        // Print measured
        printf("%f, %f, %f, %d\n", 
        gps_sensor.xPosition(distanceUnits::in) + 72,
        gps_sensor.yPosition(distanceUnits::in) + 72, 
        gps_sensor.heading(), 
        gps_sensor.quality());

        if(++rot_index > 3)
        {
            rot_index = 0;
            if (++x_tile > 5)
            {
                x_tile = 1;
                y_tile++;
            }
        }
    });
    // ================ INIT ================
    while (true)
    {
#ifdef Tank
        double l = con.Axis3.position() / 100.0;
        double r = con.Axis2.position() / 100.0;
        drive_sys.drive_tank(l, r);
#else

        double f = con.Axis3.position() / 100.0;
        double s = con.Axis1.position() / 100.0;
        drive_sys.drive_arcade(f, s);
#endif

        pose_t odom_gps_pose = odom_gps.get_position();
        printf("X: %.2f, Y: %.2f, R: %.2f, A: %.2f \n", odom_gps_pose.x, odom_gps_pose.y, odom_gps_pose.rot, odom_gps.get_alpha());

        vexDelay(10);
    }

    // ================ PERIODIC ================
}