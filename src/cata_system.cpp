#include "cata_system.h"

int thread_func(void *void_cata)
{
    CataSys &cata = *(CataSys *)void_cata;

    CataCommand last_cmd = CataCommand::StopIntake;

    const double cata_lower_threshold = 0.0;
    const double cata_upper_threshold = 1.0;

    while (true)
    {
        // read sensors
        const bool ball_in_intake = cata.intake_watcher.isNearObject();
        const bool ball_in_cata = cata.cata_watcher.isNearObject();

        if (ball_in_cata && ball_in_intake)
        {
            printf("DQed, ball in intake and catapault (or a sensor got funny)\n");
        }
        const double cata_pos = cata.cata_enc.position(vex::degrees);
        const bool cata_ready = cata_pos > cata_lower_threshold && cata_pos < cata_upper_threshold;

        // get command
        const CataCommand this_command = cata.cmd;

        // decide accordingly
        if (!cata_ready)
        {
            // spin cata in the right direction
            // don't even read that command
            cata.cata_motor.spin(vex::fwd, 3.0, vex::volt);
            continue;
        } else {
            cata.cata_motor.stop();
        }
        
        if (this_command == CataCommand::Fire){
            // spin cata to fire
        } else if (this_command == CataCommand::StopIntake){
            cata.intake_motor.stop();
        } else {
            // until i get acess to a robot
        }

        last_cmd = this_command;

        vexDelay(20);
    }
    return 0;
}

CataSys::CataSys(vex::optical &intake_watcher,
                 CustomEncoder &cata_enc,
                 vex::optical &cata_watcher,
                 vex::motor_group &cata_motor,
                 vex::motor_group &intake_motor) : intake_watcher(intake_watcher),
                                                   cata_enc(cata_enc),
                                                   cata_watcher(cata_watcher),
                                                   cata_motor(cata_motor),
                                                   intake_motor(intake_motor)
{
    runner = vex::task(thread_func, (void *)this);
}
