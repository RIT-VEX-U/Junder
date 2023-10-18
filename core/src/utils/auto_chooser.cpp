#include "../core/include/utils/auto_chooser.h"

/**
 * Initialize the auto-chooser. This class places a choice menu on the brain screen,
 * so the driver can choose which autonomous to run.
 * @param brain the brain on which to draw the selection boxes
 */
AutoChooser::AutoChooser(std::vector<std::string> paths)
{
  const static int per_line = 3;
  const static int num_lines = 2;
  const static int x_padding = 20;
  const static int y_padding = 20;

  const int entry_height = ((height - (y_padding * (num_lines - 1))) / num_lines);
  const int entry_width = ((width - (x_padding * (per_line - 1))) / per_line);

  list = std::vector<entry_t>(paths.size());
  int x = 50;
  int y = 10;
  for (size_t i = 0; i < list.size(); i++)
  {
    Rect r = Rect::from_min_and_size({(double)x, (double)y}, {entry_width, entry_height});
    list[i] = entry_t{r, paths[i]};
    x += entry_width + x_padding;
    if ((i + 1) % per_line == 0)
    {
      y += entry_height + y_padding;
      x = 50;
    }
  }
}
void AutoChooser::update(bool was_pressed, int x, int y)
{
  for (const entry_t &e : list)
  {
    if (e.rect.contains({(double)x, (double)y}))
    {
      choice = e.name;
    }
  }
}

void AutoChooser::draw(vex::brain::lcd &scr, bool first_draw, unsigned int frame_number)
{
  scr.setFont(vex::fontType::mono20);

  for (size_t i = 0; i < list.size(); i++)
  {
    entry_t e = list[i];
    scr.setFillColor(vex::blue);

    if (choice == e.name)
    {
      scr.setFillColor(vex::green);
    }
    scr.drawRectangle(e.rect.min.x, e.rect.min.y, e.rect.width(), e.rect.height());

    int width = scr.getStringWidth(e.name.c_str());
    scr.printAt(e.rect.center().x - width / 2, e.rect.center().y - 10, e.name.c_str());
  }
}

/**
 * Return the selected autonomous
 */
std::string AutoChooser::get_choice()
{
  return choice;
}