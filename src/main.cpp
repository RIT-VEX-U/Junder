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


    comp.autonomous(autonomous);
    comp.drivercontrol(autonomous);

    robot_init();

    while (1)
    {

        // Allow other tasks to run
        this_thread::sleep_for(1000);
    }
}
