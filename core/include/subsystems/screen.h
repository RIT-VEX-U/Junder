#pragma once
#include "vex.h"
#include <vector>
#include <functional>
#include <map>
#include <cassert>
#include "../core/include/subsystems/odometry/odometry_base.h"
#include "../core/include/utils/graph_drawer.h"

namespace screen
{
    /// @brief Page describes one part of the screen slideshow
    class Page
    {
    public:
        /**
         * @brief collect data, respond to screen input, do fast things (runs at
         * 50hz even if you're not focused on this Page (only drawn page gets
         * touch updates))
         * @param was_pressed true if the screen has been pressed
         * @param x x position of screen press (if the screen was pressed)
         * @param y y position of screen press (if the screen was pressed)
         */
        virtual void update(bool was_pressed, int x, int y);
        /**
         * @brief draw stored data to the screen (runs at 10 hz and only runs if
         * this page is in front)
         * @param first_draw true if we just switched to this page
         * @param frame_number frame of drawing we are on (basically an animation
         * tick)
         */
        virtual void draw(vex::brain::lcd &screen, bool first_draw,
                          unsigned int frame_number);
    };

    /// Widget that updates a double value. Updates by reference so watch out for race conditions cuz the screen stuff lives on another thread
    class SliderWidget
    {
    public:
        /// @brief Creates a slider widget
        /// @param val reference to the value to modify
        /// @param low minimum value to go to
        /// @param high maximum value to go to
        /// @param rect rect to draw it
        /// @param name name of the value
        SliderWidget(double &val, double low, double high, Rect rect, std::string name) : value(val), low(low), high(high), rect(rect), name(name) {}

        /// @brief responds to user input
        /// @param was_pressed if the screen is pressed
        /// @param x x position if the screen was pressed
        /// @param y y position if the screen was pressed
        bool update(bool was_pressed, int x, int y);
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number);

    private:
        double &value;

        double low;
        double high;

        Rect rect;
        std::string name = "";
    };

    /// Widget that updates a double value. Updates by reference so watch out for race conditions cuz the screen stuff lives on another thread
    class ButtonWidget
    {
    public:
        ButtonWidget(std::function<void(void)> onpress, Rect rect, std::string name) : onpress(onpress), rect(rect), name(name) {}
        ButtonWidget(void (*onpress)(), Rect rect, std::string name) : onpress(onpress), rect(rect), name(name) {}

        /// @brief responds to user input
        /// @param was_pressed if the screen is pressed
        /// @param x x position if the screen was pressed
        /// @param y y position if the screen was pressed
        bool update(bool was_pressed, int x, int y);
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number);

    private:
        std::function<void(void)> onpress;
        Rect rect;
        std::string name = "";
        bool was_pressed_last = false;
    };

    /**
     * @brief Start the screen background task. Once you start this, no need to draw to the screen manually elsewhere
     * @param screen reference to the vex screen
     * @param pages drawing pages
     * @param first_page optional, which page to start the program at. by default 0
     */
    void start_screen(vex::brain::lcd &screen, std::vector<Page *> pages, int first_page = 0);

    void stop_screen();

    /// @brief  type of function needed for update
    using update_func_t = std::function<void(bool, int, int)>;

    /// @brief  type of function needed for draw
    using draw_func_t = std::function<void(vex::brain::lcd &screen, bool, unsigned int)>;

    class StatsPage : public Page
    {
    public:
        StatsPage(std::map<std::string, vex::motor &> motors);
        void update(bool was_pressed, int x, int y) override;
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

    private:
        void draw_motor_stats(const std::string &name, vex::motor &mot, unsigned int frame, int x, int y, vex::brain::lcd &scr);

        std::map<std::string, vex::motor &> motors;
        static const int y_start = 0;
        static const int per_column = 4;
        static const int row_height = 20;
        static const int row_width = 200;
    };

    /**
     * @brief a page that shows odometry position and rotation and a map (if an sd card with the file is on)
     */
    class OdometryPage : public Page
    {
    public:
        OdometryPage(OdometryBase &odom, double width, double height, bool do_trail);
        void update(bool was_pressed, int x, int y) override;
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

    private:
        static const int path_len = 40;
        static constexpr char const *field_filename = "vex_field_240p.png";

        OdometryBase &odom;
        double width;
        double height;
        uint8_t *buf = nullptr;
        int buf_size = 0;
        pose_t path[path_len];
        int path_index = 0;
        bool do_trail;
    };

    /// @brief Simple page that stores no internal data. the draw and update functions use only global data rather than storing anything
    class FunctionPage : public Page
    {
    public:
        FunctionPage(update_func_t update_f, draw_func_t draw_t);

        void update(bool was_pressed, int x, int y) override;
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

    private:
        update_func_t update_f;
        draw_func_t draw_f;
    };

    class PIDPage : public Page
    {
    public:
        PIDPage(
            PID::pid_config_t &m_cfg, PID *pid, std::string name, std::function<void(void)> onchange = []() {})
            : cfg(m_cfg), pid(pid), name(name), onchange(onchange),
              p_slider(cfg.p, 0.0, 0.5, Rect{{60, 20}, {210, 60}}, "P"),
              i_slider(cfg.i, 0.0, 0.05, Rect{{60, 80}, {180, 120}}, "I"),
              d_slider(cfg.d, 0.0, 0.05, Rect{{60, 140}, {180, 180}}, "D"),
              zero_i([this]()
                     { zero_i_f(); },
                     Rect{{180, 80}, {220, 120}}, "0"),
              zero_d([this]()
                     { zero_d_f(); },
                     Rect{{180, 140}, {220, 180}}, "0"),
              graph(40, -30, 120, {vex::red, vex::green}, 2)
        {
            assert(pid != nullptr);
        }
        void update(bool was_pressed, int x, int y) override;
        void draw(vex::brain::lcd &, bool first_draw, unsigned int frame_number) override;

    private:
        void zero_d_f(void) { cfg.d = 0; }
        void zero_i_f(void) { cfg.i = 0; }

        PID::pid_config_t &cfg;
        PID *pid;
        const std::string name;
        std::function<void(void)> onchange;

        SliderWidget p_slider;
        SliderWidget i_slider;
        SliderWidget d_slider;
        ButtonWidget zero_i;
        ButtonWidget zero_d;

        GraphDrawer graph;
    };

}
