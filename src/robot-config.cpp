#include "robot-config.h"
#include "core.h"

vex::brain brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors


// ================ OUTPUTS ================
// Motors


// ================ SUBSYSTEMS ================


// ================ UTILS ================


/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
*/
void robot_init()
{
    CommandController cc;
    cc.add(new DelayCommand(100));
}
