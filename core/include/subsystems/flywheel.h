#pragma once

#include "../core/include/utils/controls/feedforward.h"
#include "vex.h"
#include "../core/include/robot_specs.h"
#include "../core/include/utils/controls/pid.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/screen.h"
#include <atomic>

/**
 * a Flywheel class that handles all control of a high inertia spinning disk
 * It gives multiple options for what control system to use in order to control wheel velocity and functions alerting the user when the flywheel is up to speed.
 * Flywheel is a set and forget class.
 * Once you create it you can call spin_rpm or stop on it at any time and it will take all necessary steps to accomplish this
 *
 */

template <typename Controller, typename Filter>
int spinRPMTask(void *wheelPointer);

template <typename Controller, typename Filter>
class FlywheelPage;

template <typename Controller, typename Filter = MovingAverage>
class Flywheel
{
  // using namespace vex;
  static_assert(std::is_convertible<Controller *, Feedback *>::value, "Controller should implement the Feedback interface");
  static_assert(std::is_convertible<Filter *, FilterBase *>::value, "Filtetr should implement the FilterBase interface");

  template<typename tC, typename tF>
  friend int spinRPMTask(void *wheelPointer);

public:
  // CONSTRUCTORS, GETTERS, AND SETTERS
  /**
   * Create the Flywheel object using PID + feedforward for control.
   * @param motors      pointer to the motors on the fly wheel
   * @param feedback    a feedback controleller
   * @param ratio       ratio of the gears from the motor to the flywheel just multiplies the velocity
   * @param moving_avg_size this size of the moving average window
   */
  Flywheel(motor_group &motors, Controller &con, Filter &filt, FeedForward &helper, const double ratio) : motors(motors),
                                                                                                          task_running(false), fb(con), ff(helper),
                                                                                                          ratio(ratio), avger(filt) {}

  /**
   * Return the target_rpm that the flywheel is currently trying to achieve
   * @return target_rpm  the target rpm
   */
  double get_target() const
  {
    return target_rpm;
  }

  /**
   * return the velocity of the flywheel
   */
  double getRPM() const
  {
    return avger.get_average();
  }

  /**
   * Returns the motors
   */
  vex::motor_group &get_motors() const
  {
    return motors;
  }

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY OPCONTROL AND AUTONOMOUS - this only applies if the target_rpm thread is not running
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_manual(double speed, directionType dir = fwd)
  {
    if (!task_running)
      motors.spin(dir, speed * 12, voltageUnits::volt);
  }

  /**
   * starts or sets the target_rpm thread at new value
   * what control scheme is dependent on control_style
   * @param rpm - the target_rpm we want to spin at
   */
  void spin_rpm(double input_rpm)
  {
    // setting to 0 is equivelent to stopping
    if (input_rpm == 0.0)
    {
      stop();
    }
    // only run if the RPM is different or it isn't already running
    if (!task_running)
    {
      rpm_task = task(spinRPMTask<Controller, Filter>, this);
      task_running = true;
    }
    // now that its running>, set the target
    set_target(input_rpm);
  }

  /**
   * Stops the motors. If manually spinning, this will do nothing just call spin_mainual(0.0) to send 0 volts
   */
  void stop()
  {
    if (task_running)
    {
      task_running = false;
      rpm_task.stop();
      target_rpm = 0.0;
      motors.stop();
    }
  }

  bool is_on_target()
  {
    return fb.is_on_target();
  }

  /// @brief Creates a page displaying info about the flywheel
  /// @return the page should be used for `screen::start_screen(screen, {fw.Page()});
  screen::Page *Page() const
  {
    return new FlywheelPage<Controller, Filter>(*this);
  }

  /// @brief Creates a new auto command to spin the flywheel at the desired velocity
  /// @param rpm the rpm to spin at
  /// @return an auto command to add to a command controller
  AutoCommand *SpinRpmCmd(int rpm)
  {

    return new FunctionCommand([this, rpm]()
                               {spin_rpm(rpm); return true; });
  }

  /// @brief Creates a new auto command that will hold until the flywheel has its target as defined by its feedback controller
  /// @return an auto command to add to a command controller
  AutoCommand *WaitUntilUpToSpeedCmd()
  {
    return new WaitUntilCondition(
        new FunctionCondition([this]()
                              { return is_on_target(); }));
  }

private:
  template <typename pC, typename pF>
  friend class FlywheelPage;
  vex::motor_group &motors;       ///< motors that make up the flywheel
  bool task_running = false;      ///< is the task currently running?
  Controller &fb;                 ///< Main Feeback controller
  FeedForward &ff;                ///< Helper Feedforward Controller
  vex::mutex fb_mut;              ///< guard for talking to the runner thread
  double ratio;                   ///< ratio between motor and flywheel. For accurate RPM calcualation
  std::atomic<double> target_rpm; ///< Desired RPM of the flywheel.
  task rpm_task;                  ///< task that handles spinning the wheel at a given target_rpm
  Filter avger;                   ///< Moving average to smooth out noise from

  // Functions for internal use only
  /**
   * Sets the target rpm of the flywheel
   * @param value - desired RPM
   */
  void set_target(double value)
  {
    fb_mut.lock();
    target_rpm = (value);
    fb.init(getRPM(), value);
    fb_mut.unlock();
  }
  /**
   * make a measurement of the current target_rpm of the flywheel motor and return a smoothed version
   */
  double measure_RPM()
  {
    double rawRPM = ratio * motors.velocity(velocityUnits::rpm);
    avger.add_entry(rawRPM);
    return avger.get_average();
  }

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY TASKS ONLY
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_raw(double speed, directionType dir = fwd)
  {
    motors.spin(dir, speed * 12, voltageUnits::volt);
  }
};

template <typename Controller, typename Filter>
class FlywheelPage : public screen::Page
{
public:
  static const size_t window_size = 40;

  FlywheelPage(const Flywheel<Controller, Filter> &fw) : fw(fw), gd(GraphDrawer(window_size, 0.0, 0.0, {vex::color(255, 0, 0), vex::color(0, 255, 0), vex::color(0, 0, 255)}, 3)), avg_err(window_size) {}
  /// @brief @see Page#update
  void update(bool, int, int) override {}
  /// @brief @see Page#draw
  void draw(vex::brain::lcd &screen, bool,
            unsigned int) override
  {

    double target = fw.get_target();
    double actual = fw.getRPM();
    double err = fabs(target - actual);

    avg_err.add_entry(err);
    double volts = fw.fb.get() * 12.0;
    gd.add_samples({target, actual, volts / 12.0 * 1000.0});

    gd.draw(screen, 200, 10, 220, 220);
    screen.setPenColor(vex::white);
    screen.printAt(50, 30, "set: %.2f", target);
    screen.printAt(50, 60, "act: %.2f", actual);
    screen.printAt(50, 90, "stddev: %.2f", avg_err.get_average());
    screen.printAt(50, 150, "temp: %.2fc", fw.get_motors().temperature(vex::celsius));
    screen.printAt(50, 180, "volt: %.2fv", volts);
  }

private:
  const Flywheel<Controller, Filter> &fw;
  GraphDrawer gd;
  MovingAverage avg_err;
};

template <typename Controller, typename Filter>
int spinRPMTask(void *wheelPointer)
{
  Flywheel<Controller, Filter> &wheel = *(Flywheel<Controller, Filter> *)wheelPointer;

  // get the pid from the wheel and set its target to the RPM stored in the wheel.
  while (true)
  {
    double rpm = wheel.measure_RPM();

    if (wheel.target_rpm != 0)
    {
      double output = wheel.ff.calculate(wheel.target_rpm, 0.0, 0.0);
      {
        wheel.fb_mut.lock();
        wheel.fb.update(rpm); // check the current velocity and update the PID with it.

        output += wheel.fb.get();
        wheel.fb_mut.unlock();
      }

      wheel.spin_raw(output, fwd); // set the motors to whatever feedforward tells them to do
    }
    vexDelay(5);
  }
  return 0;
}
