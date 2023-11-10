#pragma once
#include "vex.h"
/**
 * Main entrypoint for the autonomous period
*/
// drive commands
#define DRIVE_TO_POINT_FAST(x,y,dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, x, y, directionType::dir))
#define DRIVE_TO_POINT_SLOW(x,y,dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, x, y, directionType::dir))
#define DRIVE_FORWARD_FAST(in, dir) (new DriveForwardCommand(drive_sys, drive_fast_mprofile, in, directionType::dir))
#define DRIVE_FORWARD_SLOW(in, dir) (new DriveForwardCommand(drive_sys, drive_slow_mprofile, in, directionType::dir))
#define DRIVE_TO_POINT_SLOW_PT(pt, dir) (new DriveToPointCommand(drive_sys, drive_slow_mprofile, pt, dir))
#define DRIVE_TO_POINT_FAST_PT(pt, dir) (new DriveToPointCommand(drive_sys, drive_fast_mprofile, pt, dir))

// turn commands
#define TURN_TO_HEADING(dir) (new TurnToHeadingCommand(drive_sys, *robot_cfg.turn_feedback, dir, TURN_SPEED))
#define TURN_DEGREES(dir) (new TurnDegreesCommand(drive_sys, *robot_cfg.turn_feedback, dir, TURN_SPEED))
#define TURN_TO_POINT(pt) (new TurnToPointCommand(drive_sys, odom, *config.turn_feedback, pt))

void autonomous();