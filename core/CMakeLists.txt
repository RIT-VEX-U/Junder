
cmake_minimum_required(VERSION 3.11)

set(CMAKE_TOOLCHAIN_FILE /home/richie/Clubs/VEX/BuildSystem/Toolchain_VexV5.cmake)

project(Core LANGUAGES C CXX)

set(core_headers
    include/core/utils/graph_drawer.h
    include/core/utils/vector2d.h
    include/core/utils/geometry.h
    include/core/utils/controls
    include/core/utils/controls/take_back_half.h
    include/core/utils/controls/feedforward.h
    include/core/utils/controls/pid.h
    include/core/utils/controls/pidff.h
    include/core/utils/controls/trapezoid_profile.h
    include/core/utils/controls/feedback_base.h
    include/core/utils/controls/motion_controller.h
    include/core/utils/controls/bang_bang.h
    include/core/utils/serializer.h
    include/core/utils/moving_average.h
    include/core/utils/command_structure
    include/core/utils/command_structure/basic_command.h
    include/core/utils/command_structure/auto_command.h
    include/core/utils/command_structure/command_controller.h
    include/core/utils/command_structure/flywheel_commands.h
    include/core/utils/command_structure/drive_commands.h
    include/core/utils/command_structure/delay_command.h
    include/core/utils/generic_auto.h
    include/core/utils/logger.h
    include/core/utils/math_util.h
    include/core/utils/auto_chooser.h
    include/core/utils/pure_pursuit.h
    include/core/subsystems/lift.h
    include/core/subsystems/odometry
    include/core/subsystems/odometry/odometry_3wheel.h
    include/core/subsystems/odometry/odometry_base.h
    include/core/subsystems/odometry/odometry_tank.h
    include/core/subsystems/mecanum_drive.h
    include/core/subsystems/tank_drive.h
    include/core/subsystems/fun/pl_mpeg.h
    include/core/subsystems/fun/video.h
    include/core/subsystems/custom_encoder.h
    include/core/subsystems/screen.h
    include/core/subsystems/flywheel.h
    include/core/subsystems/layout.h
    include/core/robot_specs.h
)
set(core_sources
    src/utils/vector2d.cpp
    src/utils/pure_pursuit.cpp
    src/utils/graph_drawer.cpp
    src/utils/math_util.cpp
    src/utils/controls/feedforward.cpp
    src/utils/controls/bang_bang.cpp
    src/utils/controls/motion_controller.cpp
    src/utils/controls/pidff.cpp
    src/utils/controls/take_back_half.cpp
    src/utils/controls/pid.cpp
    src/utils/generic_auto.cpp
    src/utils/moving_average.cpp
    src/utils/trapezoid_profile.cpp
    src/utils/command_structure
    src/utils/command_structure/auto_command.cpp
    src/utils/command_structure/command_controller.cpp
    src/utils/command_structure/basic_command.cpp
    src/utils/command_structure/flywheel_commands.cpp
    src/utils/command_structure/drive_commands.cpp
    src/utils/serializer.cpp
    src/utils/auto_chooser.cpp
    src/utils/logger.cpp
    src/subsystems/odometry
    src/subsystems/odometry/odometry_3wheel.cpp
    src/subsystems/odometry/odometry_base.cpp
    src/subsystems/odometry/odometry_tank.cpp
    src/subsystems/flywheel.cpp
    src/subsystems/custom_encoder.cpp
    src/subsystems/screen.cpp
    src/subsystems/mecanum_drive.cpp
    src/subsystems/fun/video.cpp
    src/subsystems/tank_drive.cpp
)

add_library(Core OBJECT
	${core_sources}
    ${core_headers}
     include/core/vex.h
)
target_include_directories(Core PUBLIC include)
target_compile_options(Core PUBLIC -Wno-c++17-extensions -Wno-c++14-extensions)

