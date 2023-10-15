/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Ryan McGee                                                */
/*    Created:      9/10/2023, 12:37:29 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

#include "competition/autonomous.h"
#include "competition/opcontrol.h"
#include "robot-config.h"

using namespace vex;

vex::competition comp;

/**
 * Program entrypoint. Defines the competition autonomous and opcontrol entrypoints,
 * and init functions.
 *
 * Do not modify this file!
 */

int main()
{
    printf("ma \t ema \t sensor\n");
    ExponentialMovingAverage ema{4};
    MovingAverage ma{4};
    for (int j = 0; j < 20; j++)
    {
        int i = j;
        if (j > 10)
        {
            i = 10;
        }
        ema.add_entry((double)i);
        ma.add_entry((double)i);
        printf("%f \t %f \t %f\n", ma.get_average(), ema.get_average(), (double)i);
        fflush(stdout);
    }

    comp.autonomous(autonomous);
    comp.drivercontrol(opcontrol);

    robot_init();

    while (1)
    {

        // Allow other tasks to run
        this_thread::sleep_for(1000);
    }
}
