#include "vex.h"
#include <mutex>
#include <atomic>
#include "../core/include/subsystems/custom_encoder.h"

enum class CataCommand
{
    Intake,       // all mutually exclusive or else we get DQed or jam the cata
    IntakeToHold, // all mutually exclusive or else we get DQed or jam the cata
    Fire,          // all mutually exclusive or else we get DQed or jam the cata
    StopIntake
};

class CataSys
{
    public:
    friend int thread_func(void *void_cata);

    CataSys(vex::optical &intake_watcher, CustomEncoder &cata_enc, vex::optical &cata_watcher, vex::motor_group &cata_motor, vex::motor_group &intake_motor);
    void SendCommand(CataCommand cmd);
private:
    vex::optical &intake_watcher;
    CustomEncoder &cata_enc;
    vex::optical &cata_watcher;
    vex::motor_group &cata_motor;
    vex::motor_group &intake_motor;

    vex::task runner;
    std::atomic<CataCommand> cmd;
};