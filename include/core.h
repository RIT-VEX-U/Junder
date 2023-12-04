#pragma once

// Subsystems package
#include "core/subsystems/odometry/odometry_3wheel.h"
#include "core/subsystems/odometry/odometry_base.h"
#include "core/subsystems/odometry/odometry_tank.h"
#include "core/subsystems/custom_encoder.h"
#include "core/subsystems/flywheel.h"
#include "core/subsystems/lift.h"
#include "core/subsystems/mecanum_drive.h"
#include "core/subsystems/tank_drive.h"
#include "core/subsystems/screen.h"

// Utils package
#include "core/utils/command_structure/auto_command.h"
#include "core/utils/command_structure/basic_command.h"
#include "core/utils/command_structure/command_controller.h"
#include "core/utils/command_structure/delay_command.h"
#include "core/utils/command_structure/drive_commands.h"
#include "core/utils/command_structure/flywheel_commands.h"

#include "core/utils/auto_chooser.h"
#include "core/utils/generic_auto.h"
#include "core/utils/geometry.h"
#include "core/utils/graph_drawer.h"
#include "core/utils/math_util.h"
#include "core/utils/moving_average.h"


#include "core/utils/controls/feedback_base.h"
#include "core/utils/controls/feedforward.h"
#include "core/utils/controls/pid.h"
#include "core/utils/controls/pidff.h"
#include "core/utils/controls/bang_bang.h"
#include "core/utils/controls/take_back_half.h"

#include "core/utils/controls/motion_controller.h"



#include "core/utils/controls/trapezoid_profile.h"
#include "core/utils/pure_pursuit.h"
#include "core/utils/vector2d.h"
#include "core/utils/serializer.h"

// Base package
#include "core/robot_specs.h"