#pragma once
#include "vex.h"
#include "core.h"

extern vex::brain brain;
extern vex::controller con;
extern vex::inertial imu;

// ================ INPUTS ================
// Digital sensors

// Analog sensors


// ================ OUTPUTS ================
// Motors

// ================ SUBSYSTEMS ================
extern robot_specs_t robot_cfg;
extern OdometryTank odom;
extern TankDrive drive_sys;

// ================ UTILS ================

void robot_init();