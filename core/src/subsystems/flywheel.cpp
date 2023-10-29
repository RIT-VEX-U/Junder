#include "../core/include/subsystems/flywheel.h"
#include "../core/include/utils/controls/feedforward.h"
#include "../core/include/utils/controls/pid.h"
#include "../core/include/utils/math_util.h"
#include "../core/include/subsystems/screen.h"
#include "../core/include/utils/graph_drawer.h"
#include "vex.h"

using namespace vex;


/**
 * Runs a thread that keeps track of updating flywheel RPM and controlling it accordingly
 */


//------------------------- Screen Stuff ----------------------------
