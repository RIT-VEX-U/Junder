
cmake_minimum_required(VERSION 3.11)

set(CMAKE_TOOLCHAIN_FILE /home/richie/Clubs/VEX/BuildSystem/Toolchain.cmake)

project(Core LANGUAGES C CXX)

add_compile_options(-Wno-c++17-extensions -Wno-c++14-extensions)

vex_add_executable(Junder)

set(sources
    src/automation.cpp
    src/cata_system.cpp
    src/main.cpp
    src/robot-config.cpp
    src/competition
    src/competition/opcontrol.cpp
    src/competition/autonomous.cpp
    include/vex.h
    include/robot-config.h
    include/core.h
    include/competition
    include/competition/opcontrol.h
    include/competition/autonomous.h
    include/cata_system.h
    include/automation.h

)
target_sources(Junder PRIVATE ${sources})

target_include_directories(Junder PRIVATE include)

add_subdirectory(core/)

target_link_libraries(Junder Core)
