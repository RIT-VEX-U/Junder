#include "vex.h"
#include <mutex>
#include <atomic>
#include "../core/include/subsystems/custom_encoder.h"
#include "../core/include/utils/command_structure/auto_command.h"
#include "../core/include/subsystems/screen.h"

class CataSys
{
public:
    enum class Command
    {
        Intake,       // all mutually exclusive or else we get DQed or jam the cata
        IntakeToHold, // all mutually exclusive or else we get DQed or jam the cata
        Fire,         // all mutually exclusive or else we get DQed or jam the cata
        StopIntake
    };
    struct State
    {
        bool ball_in_intake;
        bool ball_in_cata;
        bool cata_in_position;
    };

    CataSys(vex::optical &intake_watcher, CustomEncoder &cata_enc, vex::optical &cata_watcher, vex::motor_group &cata_motor, vex::motor_group &intake_motor);
    void send_command(Command cmd);
    State get_state() const;

    // Autocommands
    AutoCommand *Fire();
    AutoCommand *IntakeToHold();
    AutoCommand *IntakeFully();

    // Page
    screen::Page *Page();

private:
    // configuration
    vex::optical &intake_watcher;
    CustomEncoder &cata_enc;
    vex::optical &cata_watcher;
    vex::motor_group &cata_motor;
    vex::motor_group &intake_motor;

    // running
    vex::task runner;
    std::atomic<Command> cmd;
    std::atomic<State> state;
    friend int thread_func(void *void_cata);
};