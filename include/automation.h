#pragma once
#include "core.h"
#include <functional>

// ================ Autonomous Abstractions ================
AutoCommand *WingSetCmd(bool val);

// ================ Autonomous Abstractions ================
struct vision_filter_s {
    int min_area;

    double aspect_low;
    double aspect_high;

    double min_x;
    double max_x;
    double min_y;
    double max_y;
};

extern vision_filter_s default_vision_filter;

class VisionTrackTriballCommand : public AutoCommand {
  public:
    VisionTrackTriballCommand(vision_filter_s &filter = default_vision_filter);
    bool run() override;

  private:
    PIDFF angle_fb;
    vision_filter_s &filter;
};

class VisionObjectExists : public Condition {
  public:
    VisionObjectExists(vision_filter_s &filter = default_vision_filter);
    bool test() override;

  private:
    vision_filter_s &filter;
};

std::vector<vision::object>
vision_run_filter(vision::signature &sig,
                  vision_filter_s &filter = default_vision_filter);

point_t estimate_triball_pos(vision::object &obj);

class IsTriballInArea : public Condition {
  public:
    IsTriballInArea(point_t pos, int radius,
                    vision_filter_s &filter = default_vision_filter);
    bool test() override;

  private:
    vision_filter_s &filter;
    point_t pos;
    int radius;
};

// ================ Driver Assist Automations ================
// void matchload_1(bool &enable);
void matchload_1(std::function<bool()> enable);

AutoCommand *ClimbBarDeploy();
AutoCommand *WingSetCmd(bool val);

AutoCommand *Climb();