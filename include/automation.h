#pragma once
#include "core.h"
#include <functional>

// ================ Autonomous Abstractions ================

// ================ Driver Assist Automations ================
// void matchload_1(bool &enable);
void matchload_1(std::function<bool()> enable);

AutoCommand *ClimbBarDeploy();
AutoCommand *WingSetCmd(bool val);

AutoCommand *Climb();