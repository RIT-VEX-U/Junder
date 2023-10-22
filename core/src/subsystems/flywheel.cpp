/*********************************************************
 *
 *     File:     Flywheel.cpp
 *     Purpose:  Generalized flywheel class for Core.
 *     Author:   Chris Nokes, Richie Sommers
 *
 **********************************************************
 * EDIT HISTORY
 **********************************************************
 * 09/16/2022  <CRN> Created file, added constructor, spins, target_rpm setting, stop.
 * 09/18/2022  <CRN> Added async functionality.
 * 09/22/2022  <CRN> Documentation improvements, fixed error if target_rpm is set but motor is stopped.
 * 09/23/2022  <CRN> Neatened up program, added getters and setters, fixed documentation and bang bang.
 * 09/29/2022  <CRN> Bug fixes, target_rpm handling. Multiplied the motor by 18.
 *********************************************************/

#include "../core/include/subsystems/flywheel.h"
#include "../core/include/utils/controls/feedforward.h"
#include "../core/include/utils/controls/pid.h"
#include "../core/include/utils/math_util.h"
#include "vex.h"

using namespace vex;

/*********************************************************
 *         CONSTRUCTOR, GETTERS, SETTERS
 *********************************************************/
// used when using bang bang or TBH control
PID::pid_config_t empty_pid = PID::pid_config_t{};

Flywheel::Flywheel(motor_group &motors, Feedback &feedback, FeedForward &helper, const double ratio, const size_t moving_avg_size) : motors(motors),
                                                                                                                                     taskRunning(false), fb(feedback), ff(helper),
                                                                                                                                     ratio(ratio), avger(moving_avg_size) {}

/**
 * Return the current value that the target_rpm should be set to
 */
double Flywheel::getTargetRPM() { return target_rpm; }

/**
 * Checks if the background target_rpm controlling task is running
 * @return taskRunning - If the task is running
 */
bool Flywheel::isTaskRunning() { return taskRunning; }

/**
 * Returns a POINTER TO the motors; not currently used.
 * @return motorPointer -pointer to the motors
 */
motor_group &Flywheel::getMotors() { return motors; } // TODO -- Remove?

/**
 * return the current velocity of the flywheel motors, in RPM
 * @return the measured velocity of the flywheel
 */
double Flywheel::measureRPM()
{
  double rawRPM = ratio * motors.velocity(velocityUnits::rpm);
  avger.add_entry(rawRPM);
  return avger.get_average();
}

double Flywheel::getRPM()
{
  return avger.get_average();
}

/*********************************************************
 *         RPM SETTING THREADS
 * ALL OF THE FOLLOWING PROGRAMS HAVE THE SAME PARAMETERS AND RESULTS:
 * spin this flywheel at a given RPM, async; runs until stop(), stopThread(), or a new spinRPM() is called.
 * @param wheelPointer - points to the current wheel object
 *********************************************************/

/**
 * Runs a Feedforward variant to control rpm
 */
int spinRPMTask(void *wheelPointer)
{
  Flywheel &wheel = *(Flywheel *)wheelPointer;

  printf("starting task\n");

  // get the pid from the wheel and set its target to the RPM stored in the wheel.
  while (true)
  {
    if (wheel.target_rpm != 0)
    {
      double output = wheel.ff.calculate(wheel.target_rpm, 0.0, 0.0);
      // printf("Starting output: %f\n", output);
      double rpm = wheel.measureRPM();
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
/**
 * Runs a Take Back Half variant to control RPM
 * https://www.vexwiki.org/programming/controls_algorithms/tbh
 */
// int spinRPMTask_TBH(void *wheelPointer)
// {
// Flywheel *wheel = (Flywheel *)wheelPointer;
//
// double tbh = 0.0;
// double output = 0.0;
// double previous_error = 0.0;
//
// while (true)
// {
// wheel->measureRPM();
//
// reset if set to 0, this keeps the tbh val from screwing us up when we start up again
// if (wheel->getTargetRPM() == 0)
// {
// output = 0;
// tbh = 0;
// }
//
// double error = wheel->getTargetRPM() - wheel->getRPM();
// output += wheel->getTBHGain() * error;
// wheel->spin_raw(clamp(output, 0, 1), fwd);
//
// if (sign(error) != sign(previous_error))
// {
// output = .5 * (output + tbh);
// tbh = output;
// previous_error = error;
// }
//
// vexDelay(1);
// }
//
// return 0;
// }
/*********************************************************
 *         SPINNERS AND STOPPERS
 *********************************************************/

/**
 * Spin motors using voltage; defaults forward at 12 volts
 * FOR USE BY TASKS ONLY
 * @param speed - speed (between -1 and 1) to set the motor
 * @param dir - direction that the motor moves in; defaults to forward
 */
void Flywheel::spin_raw(double speed, directionType dir)
{
  motors.spin(dir, speed * 12, voltageUnits::volt);
}

/**
 * Spin motors using voltage; defaults forward at 12 volts
 * FOR USE BY OPCONTROL AND AUTONOMOUS - this only applies if the RPM thread is not running
 * @param speed - speed (between -1 and 1) to set the motor
 * @param dir - direction that the motor moves in; defaults to forward
 */
void Flywheel::spin_manual(double speed, directionType dir)
{
  if (!taskRunning)
    motors.spin(dir, speed * 12, voltageUnits::volt);
}

/**
 * starts or sets the RPM thread at new value
 * what control scheme is dependent on control_style
 * @param inputRPM - set the current RPM
 */
void Flywheel::spinRPM(int inputRPM)
{
  // setting to 0 is equivelent to stopping
  if (inputRPM == 0.0)
  {
    printf("STOPPING\n");
    stop();
  }
  // only run if the RPM is different or it isn't already running
  if (!taskRunning)
  {
    printf("starting taskRunning");

    rpmTask = task(spinRPMTask, this);
    taskRunning = true;
  }
  setTarget(inputRPM);
}
void Flywheel::setTarget(double value)
{
  fb_mut.lock();
  target_rpm = (value);
  fb.init(getRPM(), value);
  fb_mut.unlock();
}

/**
 * stop the RPM thread and the wheel
 */
void Flywheel::stop()
{
  rpmTask.stop();
  taskRunning = false;
  target_rpm = 0.0;
  motors.stop();
}
