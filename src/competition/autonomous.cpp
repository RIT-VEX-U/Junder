#include "competition/autonomous.h"
#include "robot-config.h"
#include "core.h"
#include <functional>
/**
 * Main entrypoint for the autonomous period
 */
const double TURN_SPEED = 0.6;
static const double tile_size = 24.0;
static int x_tile = 1;
static int y_tile = 1;
static int rot_index = 0;

void autonomous()
{
    while (imu.isCalibrating() || gps_sensor.isCalibrating())
    {
        vexDelay(20);
    }

    CommandController cmd;

    cmd.add(new OdomSetPosition(odom, {.x=24, .y=24, .rot=0}));

    for (int y = 1; y < 6; y ++) {

        for (int x = 1; x < 6; x++) {

            if (!(x == 1 && y == 1)) {
                cmd.add(new DriveToPointCommand(drive_sys, *robot_cfg.drive_feedback, {.x=(x * tile_size), .y=(y * tile_size)}, vex::forward, 0.3));
            }

            cmd.add(new FunctionCommand([]() {
                // Print measured
                for (int i = 0; i < 4; i++) {

                    spinnyPID.init(gps_sensor.heading(), rot_index * 90);
                    spinnyPID.set_limits(-1, 1);
                    
                    while(!spinnyPID.is_on_target()) {
                        spinnyPID.update(gps_sensor.rotation());
                        
                        gps_spinner.spin(vex::reverse, spinnyPID.get(), vex::voltageUnits::volt);
                
                        vexDelay(20);
                    }

                    gps_spinner.stop();

                    vex::wait(1000, msec);

                    if(++rot_index > 3)
                    {
                        rot_index = 0;
                        if (++x_tile > 5)
                        {
                            x_tile = 1;
                            y_tile++;
                        }
                    }

                    // print expected
                    printf("%f, %f, %f, ",
                    x_tile * tile_size, 
                    y_tile * tile_size, 
                    rot_index * 90.0);

                    // print actual
                    printf("%f, %f, %f, %d\n", 
                    gps_sensor.xPosition(distanceUnits::in) + 72,
                    gps_sensor.yPosition(distanceUnits::in) + 72, 
                    gps_sensor.heading(), 
                    gps_sensor.quality());

                    fflush(stdout);
                }

                vex::wait(1000, msec);

                spinnyPID.init(gps_sensor.heading(), 1);
                spinnyPID.set_limits(-1, 1);
                
                while(!spinnyPID.is_on_target()) {
                    spinnyPID.update(gps_sensor.rotation());
                    
                    gps_spinner.spin(vex::reverse, spinnyPID.get(), vex::voltageUnits::volt);
            
                    vexDelay(20);
                }
                
                return true;
            }));
        }

        if (y != 5) {
            cmd.add({
                new DriveToPointCommand(drive_sys, *robot_cfg.drive_feedback, {.x=24, .y=((y+1) * tile_size)}, vex::reverse, 0.5),
                TURN_TO_HEADING(0)
            });
        }
    }

    cmd.run();
}