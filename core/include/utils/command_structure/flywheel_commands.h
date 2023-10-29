/**
 * File: flywheel_commands.h
 * Desc:
 *    [insert meaningful desc]
 */

#pragma once

#include "../core/include/subsystems/flywheel.h"
#include "../core/include/utils/command_structure/auto_command.h"

/**
 * AutoCommand wrapper class for the spin_rpm function
 * in the Flywheel class
 *
 */
template <typename Controller, typename Filter>
class SpinRPMCommand : public AutoCommand
{
public:
  /**
   * Construct a SpinRPM Command
   * @param flywheel the flywheel sys to command
   * @param rpm the rpm that we should spin at
   */
  SpinRPMCommand(Flywheel<Controller, Filter> &flywheel, int rpm) : flywheel(flywheel), rpm(rpm) {}

  /**
   * Run spin_manual
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override
  {
    flywheel.spin_rpm(rpm);
    return true;
  }

private:
  // Flywheel instance to run the function on
  Flywheel<Controller, Filter> &flywheel;

  // parameters for spin_rpm
  int rpm;
};

/**
 * AutoCommand that listens to the Flywheel and waits until it is at its target speed +/- the specified threshold
 *
 */
template <typename Controller, typename Filter>
class WaitUntilUpToSpeedCommand : public AutoCommand
{
public:
  /**
   * Creat a WaitUntilUpToSpeedCommand
   * @param flywheel the flywheel system we are commanding
   * @param threshold_rpm the threshold over and under the flywheel target RPM that we define to be acceptable
   */
  WaitUntilUpToSpeedCommand(Flywheel<Controller, Filter> &flywheel, int threshold_rpm) : flywheel(flywheel), threshold_rpm(threshold_rpm) {}
  /**
   * Run spin_manual
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override
  {
    // If we're withing the specified threshold, we're ready to fire
    if (fabs(flywheel.get_target() - flywheel.getRPM()) < threshold_rpm)
    {
      return true;
    }
    // else, keep waiting
    return false;
  }

private:
  // Flywheel instance to run the function on
  Flywheel<Controller, Filter> &flywheel;

  // if the actual speed is equal to the desired speed +/- this value, we are ready to fire
  int threshold_rpm;
};

/**
 * AutoCommand wrapper class for the stop function
 * in the Flywheel class
 *
 */
template <typename Controller, typename Filter>
class FlywheelStopCommand : public AutoCommand
{
public:
  /**
   * Construct a FlywheelStopCommand
   * @param flywheel the flywheel system we are commanding
   */
  FlywheelStopCommand(Flywheel<Controller, Filter> &flywheel) : flywheel(flywheel) {}

  /**
   * Run stop
   * Overrides run from AutoCommand
   * @returns true when execution is complete, false otherwise
   */
  bool run() override
  {
    flywheel.stop();
    return true;
  }

private:
  // Flywheel instance to run the function on
  Flywheel<Controller, Filter> &flywheel;
};

/**
 * AutoCommand wrapper class for the stopMotors function
 * in the Flywheel class
 *
 */
// template <typename Controller, typename Filter>
// class FlywheelStopMotorsCommand : public AutoCommand
// {
// public:
//   /**
//    * Construct a FlywheeStopMotors Command
//    * @param flywheel the flywheel system we are commanding
//    */
//   FlywheelStopMotorsCommand(Flywheel<Controller, Filter> &flywheel);

//   /**
//    * Run stop
//    * Overrides run from AutoCommand
//    * @returns true when execution is complete, false otherwise
//    */
//   bool run() override;

// private:
//   // Flywheel instance to run the function on
//   Flywheel<Controller, Filter> &flywheel;
// };

/**
 * AutoCommand wrapper class for the stopNonTasks function
 * in the Flywheel class
 *
 */
// template <typename Controller, typename Filter>
// class FlywheelStopNonTasksCommand : public AutoCommand
// {
//   FlywheelStopNonTasksCommand(Flywheel<Controller, Filter> &flywheel);

//   /**
//    * Run stop
//    * Overrides run from AutoCommand
//    * @returns true when execution is complete, false otherwise
//    */
//   bool run() override;

// private:
//   // Flywheel instance to run the function on
//   Flywheel<Controller, Filter> &flywheel;
// };

// template<typename Controller, typename Filter>
// FlywheelStopCommand::FlywheelStopCommand(Flywheel &flywheel)
// bool FlywheelStopCommand::run()

// template<typename Controller, typename Filter>
// FlywheelStopMotorsCommand::FlywheelStopMotorsCommand(Flywheel &flywheel):
//   flywheel(flywheel) {}

// bool FlywheelStopMotorsCommand::run() {
//   flywheel.stop();
//   return true;
// }

// template<typename Controller, typename Filter>
// FlywheelStopNonTasksCommand::FlywheelStopNonTasksCommand(Flywheel &flywheel):
//   flywheel(flywheel) {}

// template<typename Controller, typename Filter>
// bool FlywheelStopNonTasksCommand::run() {
//   flywheel.stop();
//   return true;
// }
