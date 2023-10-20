#pragma once
#include "vex.h"
#include "core.h"
// #define COMP_BOT


extern vex::brain Brain;
extern vex::controller con;

#ifdef COMP_BOT
extern vex::inertial imu;
extern vex::motor_group cata_motors;
#endif

#ifndef COMP_BOT
extern IMU imu;
#endif

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