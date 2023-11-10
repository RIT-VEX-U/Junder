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
    printf("%s", "hi");
    fflush(stdout);
    CommandController cmd{
        new OdomSetPosition(odom, {.x=24, .y=24, .rot=0}),

        new FunctionCommand([]() {
            // Print measured
            for (int i = 0; i < 4; i++) {
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

                gps_spinner.spinFor(90, vex::degrees);
            }
            return true;
        }),

        TURN_TO_HEADING(0),

        (new DriveForwardCommand(drive_sys, *robot_cfg.drive_feedback, tile_size, vex::forward, 0.3)),

    };

    cmd.run();
}