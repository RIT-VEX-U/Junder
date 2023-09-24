#include "competition/opcontrol.h"
#include "robot-config.h"
#include "vex.h"

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{
    // ================ INIT ================
    imu.calibrate();
    while(imu.isCalibrating());
    
    std::vector<point_t> path = {
        {0, 0},
        {15.7, 68},
        {10.4, 86.5},
        {-3.5, 82.3},
        {-15,77.3},
        {-22.6,98.6},
        {9.2, 122}
        // {0, 0}
    };

    odometry.set_position();
    // ================ PERIODIC ================
    while(true)
    {
        if(con.ButtonA.pressing())
        {
            drive_system.pure_pursuit(path, fwd, 12, *robot_cfg.drive_feedback, .5);
        }else
        {
            drive_system.reset_auto();
            drive_system.drive_arcade(con.Axis3.position()/100.0, con.Axis1.position()/100.0);
        }

        if(con.ButtonB.pressing())
            odometry.set_position();

        pose_t pos = odometry.get_position();
        printf("x:%f, y:%f, rot:%f\n", pos.x, pos.y, pos.rot);
        vexDelay(10);
    }
}