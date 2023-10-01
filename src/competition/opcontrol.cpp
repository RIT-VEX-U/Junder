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
    
    static std::vector<point_t> path1 = {
        {0, 0},
        {0, 24},
        {24, 24},
        {24, 48}
    };

    static std::vector<point_t> path2 = {
        {24, 48},
        {24, 24},
        {0, 24},
        {0, 0},
    };

    odometry.set_position();

    static std::atomic<bool> auto_driving(false);
    con.ButtonA.pressed([](){
        auto_driving = true;
        CommandController cmd;
        cmd.add(new PurePursuitCommand(drive_system, *robot_cfg.drive_feedback, path1, fwd, 12, .4));
        cmd.add(new PurePursuitCommand(drive_system, *robot_cfg.drive_feedback, path2, directionType::rev, 12, .4));
        cmd.run();
        auto_driving = false;
    });

    // ================ PERIODIC ================
    while(true)
    {
        if(!auto_driving)
            drive_system.drive_arcade(con.Axis3.position()/100.0, con.Axis1.position()/100.0);

        if(con.ButtonB.pressing())
            odometry.set_position();

        pose_t pos = odometry.get_position();
        printf("x:%f, y:%f, rot:%f\n", pos.x, pos.y, pos.rot);
        vexDelay(10);
    }
}