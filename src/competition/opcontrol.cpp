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
    
    std::vector<point_t> path1 = {
        {0, 0},
        {0, -24},
        {-24, -24},
        {-24, -48}
    };

    std::vector<point_t> path2 = {
        {-24, -48},
        {-24, -24},
        {0, -24},
        {0, 0},
    };

    odometry.set_position();
    // ================ PERIODIC ================
    while(true)
    {
        if(con.ButtonA.pressing())
        {
            while(!drive_system.pure_pursuit(path1, directionType::rev, 12, *robot_cfg.drive_feedback, .5));
            while(!drive_system.pure_pursuit(path2, directionType::fwd, 12, *robot_cfg.drive_feedback, .5));
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