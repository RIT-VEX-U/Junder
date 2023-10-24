#pragma once
/*********************************************************
 *
 *     File:     Flywheel.h
 *     Purpose:  Generalized flywheel class for Core.
 *     Author:   Chris Nokes
 *
 **********************************************************
 * EDIT HISTORY
 **********************************************************
 * 09/23/2022  <CRN> Reorganized, added documentation.
 * 09/23/2022  <CRN> Added functions elaborated on in .cpp.
 *********************************************************/
#include "../core/include/utils/controls/feedforward.h"
#include "vex.h"
#include "../core/include/robot_specs.h"
#include "../core/include/utils/controls/pid.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/screen.h"
#include <atomic>

using namespace vex;

/**
 * a Flywheel class that handles all control of a high inertia spinning disk
 * It gives multiple options for what control system to use in order to control wheel velocity and functions alerting the user when the flywheel is up to speed.
 * Flywheel is a set and forget class.
 * Once you create it you can call spin_rpm or stop on it at any time and it will take all necessary steps to accomplish this
 *
 */
class Flywheel
{
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
  Flywheel(motor_group &motors, Feedback &feedback, FeedForward &helper, const double ratio, const size_t moving_avg_size = 10);

  /**
   * Return the target_rpm that the flywheel is currently trying to achieve
   * @return target_rpm  the target rpm
   */
  double getTargetRPM();

  /**
   * return the velocity of the flywheel
   */
  double getRPM();

  /**
   * Returns the motors
   */
  motor_group &getMotors();

  // SPINNERS AND STOPPERS

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY OPCONTROL AND AUTONOMOUS - this only applies if the target_rpm thread is not running
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_manual(double speed, directionType dir = fwd);

  /**
   * starts or sets the target_rpm thread at new value
   * what control scheme is dependent on control_style
   * @param rpm - the target_rpm we want to spin at
   */
  void spin_rpm(double rpm);

  /**
   * Stops the motors. If manually spinning, this will do nothing just call spin_mainual(0.0) to send 0 volts
  */
  void stop();

  screen::Page *Page();

  AutoCommand *SpinRpmCmd(int rpm)
  {

    return new FunctionCommand([this, rpm]()
                               {spin_rpm(rpm); return true; });
  }

  AutoCommand *WaitUntilUpToSpeedCmd(double threshold)
  {
    return new WaitUntilCondition(
        new FunctionCondition([this, threshold]()
                              { return fabs(target_rpm - avger.get_average()) <= threshold; }));
  }

private:
  motor_group &motors;      // motors that make up the flywheel
  bool taskRunning = false; // is the task (thread but not) currently running?
  Feedback &fb;
  FeedForward &ff;
  vex::mutex fb_mut;
  double ratio;                   // multiplies the velocity by this value
  std::atomic<double> target_rpm; // Desired RPM of the flywheel.
  task rpmTask;                   // task (thread but not) that handles spinning the wheel at a given target_rpm
  MovingAverage avger;

  // Functions for internal use only
  /**
   * Sets the target rpm of the flywheel
   * @param value - desired RPM
   */
  void set_target(double value);
  /**
   * make a measurement of the current target_rpm of the flywheel motor and return a smoothed version
   */
  double measure_RPM();

  /**
   * stops controlling thread
   */
  void stop_background();

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY TASKS ONLY
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_raw(double speed, directionType dir = fwd);

  /**
   * Checks if the background target_rpm controlling task is running
   * @return true if the task is running
   */
  bool isTaskRunning();
};