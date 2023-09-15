#include "robot-config.h"
#include "core.h"

vex::brain brain;
vex::controller con;

// ================ INPUTS ================
// Digital sensors

// Analog sensors

// ================ OUTPUTS ================
// Motors

// ================ SUBSYSTEMS ================

// ================ UTILS ================

class RandomPredicate : public Conditions::Condition
{
    bool test() override
    {

        return vex::timer::system() % 2 == 0;
    }
};
/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    AutoCommand *choice_1 = new DelayCommand(99);
    AutoCommand *choice_2 = new InOrder{
        new DelayCommand(1),
        new DelayCommand(2),
    };

    TimerPredicate drive_timer = TimerPredicate(2, vex::seconds);

    CommandController cc{
        new DelayCommand(100),
        new DelayCommand(10),
        new Branch(choice_1, choice_2, new RandomPredicate()),
        drive_timer.start(), // -> AutoCommand that when run returns immediatly and starts counting down
        new DelayCommand(3),
        Branch{x, y. drive_timer.predicate()}, // -> Condition that returns true if time has passed
    };
}
