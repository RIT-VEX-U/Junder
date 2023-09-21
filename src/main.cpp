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

class Tester
{
public:
    Tester()
    {
        printf("Ctor\n");
    }
    ~Tester()
    {
        printf("Dtor\n");
    }
};
int runner(void *cmd)
{
    Tester t{};
    while (1)
    {
        printf("schmoovin\n");

        vex::wait(400, vex::msec);
    }
}
int main()
{

    vex::task t(runner, nullptr);
    vex::wait(4, vex::seconds);
    t.stop();
    printf("Stopped\n");

    vex::wait(4, vex::seconds);
    return 0;

    comp.autonomous(autonomous);
    comp.drivercontrol(opcontrol);

    robot_init();

    while (1)
    {

        // Allow other tasks to run
        this_thread::sleep_for(1000);
    }
}
