#pragma once
#include "cata_system.h"
#include "core.h"
#include "vex.h"
#define COMP_BOT

using namespace vex;

extern brain Brain;
extern controller con;

#ifdef COMP_BOT

// ================ INPUTS ================
// Digital sensors

// Analog sensors
extern inertial imu;
extern vex::distance intake_watcher;

// ================ OUTPUTS ================
// Motors
extern motor intake_combine;
extern motor intake_roller;

extern bool DONT_RUN_CATA_YOU_FOOL;

extern motor_group cata_motors;

// ================ SUBSYSTEMS ================
extern robot_specs_t robot_cfg;
extern OdometryTank odom;
extern TankDrive drive_sys;

extern vex::optical cata_watcher;

extern CataSys cata_sys;
extern vex::digital_out left_wing;
extern vex::digital_out right_wing;

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