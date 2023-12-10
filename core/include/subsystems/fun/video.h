#include "../core/include/subsystems/screen.h"
#include "pl_mpeg.h"
#include <string>

void set_video(const std::string &filename);
void video_restart();
// plays the video set by set_video()
// because of memory constraints we're limited to one video at a time
class VideoPlayer : public screen::Page {
  public:
    VideoPlayer();
    void update(bool was_pressed, int x, int y) override;

    void draw(vex::brain::lcd &screen, bool first_draw,
              unsigned int frame_number) override;
};