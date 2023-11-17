#include "vex.h"

enum class IntakeCapacity
{
    Full,
    Empty
};

enum class IntakeState
{
    Retracted,
    Extended,
};

enum class CataCapacity
{
    Full,
    Empty,
};

enum class BallPosition{
    None,
    Intake,
    Cata,
    Both, // ah heck
};

class CataSys
{   
    friend int thread_func(void * void_cata);

    CataSys(vex::optical &intake_watcher, vex::encoder &cata_enc, vex::optical &cata_watcher);


    // auto commands
    bool fire();
    bool intake_to_cata();
    bool intake_hold();

    private:
    vex::optical &intake_watcher;
    vex::encoder &cata_enc;
    vex::optical &cata_watcher;
    vex::task runner;
};