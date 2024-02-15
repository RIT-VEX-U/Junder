#include "cata/common.h"
#include "vex.h"
#include <../core/include/utils/state_machine.h>
#include <functional>
#include <string>
enum class IntakeMessage {
    Intake,
    Outtake,
    IntakeHold,
    Dropped,
    StopIntake,
    Drop,

};
enum class IntakeState {
    Stopped,
    Intaking,
    IntakingHold,
    Outtaking,
    Dropping,
    IntakeWaitForDrop,
};
std::string to_string(IntakeState s);
std::string to_string(IntakeMessage s);

class IntakeSys
    : public StateMachine<IntakeSys, IntakeState, IntakeMessage, 5, false> {
  public:
    friend struct Stopped;
    friend struct Dropping;
    friend struct Intaking;
    friend struct IntakingHold;
    friend struct Outtaking;
    friend struct IntakeWaitForDrop;

    IntakeSys(vex::distance &intake_watcher, vex::motor &intake_lower,
              vex::motor &intake_upper, std::function<bool()> can_intake,
              std::function<bool()> ball_in_cata, DropMode drop);

    bool ball_in_intake();

  private:
    vex::distance &intake_watcher;
    vex::motor &intake_lower;
    vex::motor &intake_upper;
    std::function<bool()> can_intake;
    std::function<bool()> ball_in_cata;
};
