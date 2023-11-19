#pragma once
#include "vex.h"
#include "core.h"
// #define COMP_BOT

using namespace vex;

extern brain Brain;
extern controller con;

#ifdef COMP_BOT

// ================ INPUTS ================
// Digital sensors

// Analog sensors
// extern inertial imu;

// ================ OUTPUTS ================
// Motors
extern motor intake_combine;
extern motor intake_roller;

extern motor_group cata_motors;

// ================ SUBSYSTEMS ================
extern robot_specs_t robot_cfg;
extern OdometryTank odom;
extern TankDrive drive_sys;



#else
// ================ INPUTS ================
// Digital sensors

// Analog sensors
extern inertial imu;
extern gps gps_sensor;


// ================ OUTPUTS ================
// Motors
extern motor intake_combine;
extern motor intake_roller;

// ================ SUBSYSTEMS ================
extern OdometryTank odom;
extern TankDrive drive_sys;
#endif

// ================ UTILS ================

void robot_init();