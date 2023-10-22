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
#include <atomic>

using namespace vex;


/**
 * a Flywheel class that handles all control of a high inertia spinning disk
 * It gives multiple options for what control system to use in order to control wheel velocity and functions alerting the user when the flywheel is up to speed.
 * Flywheel is a set and forget class.
 * Once you create it you can call spinRPM or stop on it at any time and it will take all necessary steps to accomplish this
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
   * Checks if the background target_rpm controlling task is running
   * @return true if the task is running
   */
  bool isTaskRunning();

  /**
   * Returns the motors
   */
  motor_group &getMotors();

  /**
   * make a measurement of the current target_rpm of the flywheel motor and return a smoothed version
   */
  double measureRPM();

  /**
   * return the current smoothed velocity of the flywheel motors, in target_rpm
   */
  double getRPM();



  // SPINNERS AND STOPPERS

  /**
   * Spin motors using voltage; defaults forward at 12 volts
   * FOR USE BY TASKS ONLY
   * @param speed - speed (between -1 and 1) to set the motor
   * @param dir - direction that the motor moves in; defaults to forward
   */
  void spin_raw(double speed, directionType dir = fwd);

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
  void spinRPM(int rpm);

  /**
   * stop the target_rpm thread and the wheel
   */
  void stop();

  /**
   * stop only the motors; exclusively for BANG BANG use
   */
  void stopMotors();

  /**
   * Stop the motors if the task isn't running - stop manual control
   */
  void stopNonTasks();

  AutoCommand *SpinRpmCmd(int rpm)
  {

    return new FunctionCommand([this, rpm]()
                               {spinRPM(rpm); return true; });
  }

  AutoCommand *WaitUntilUpToSpeedCmd()
  {
    // return new WaitUntilCondition(
    // new FunctionCondition([this]()
    // { return target_rpm == smoothedRPM; }));
  }

private:

  /**
   * Sets the target rpm of the flywheel
   * @param value - desired RPM
   */
  void setTarget(double value);


  motor_group &motors;      // motors that make up the flywheel
  bool taskRunning = false; // is the task (thread but not) currently running?
  Feedback & fb;
  FeedForward & ff;
  vex::mutex fb_mut;
  double ratio;                   // multiplies the velocity by this value
  std::atomic<double> target_rpm; // Desired RPM of the flywheel.
  task rpmTask;                   // task (thread but not) that handles spinning the wheel at a given target_rpm
  MovingAverage avger;
};