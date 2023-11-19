#include "cata_system.h"

int thread_func(void *void_cata)
{
    CataSys &cata = *(CataSys *)void_cata;

    CataSys::Command last_cmd = CataSys::Command::StopIntake;
    cata.state = {
        .ball_in_intake = false,
        .ball_in_cata = false,
        .cata_in_position = false};

    const double cata_lower_threshold = 0.0;
    const double cata_upper_threshold = 1.0;

    while (true)
    {
        // read sensors
        const double cata_pos = cata.cata_enc.position(vex::degrees);

        const CataSys::State st = {
            .ball_in_intake = cata.intake_watcher.isNearObject(),
            .ball_in_cata = cata.cata_watcher.isNearObject(),
            .cata_in_position = cata_pos > cata_lower_threshold && cata_pos < cata_upper_threshold};

        if (st.ball_in_cata && st.ball_in_intake)
        {
            printf("DQed! ball in intake and catapault (or a sensor got funny)\n");
        }
        cata.state = st;

        // get command
        const CataSys::Command this_command = cata.cmd;

        // decide accordingly
        if (!st.cata_in_position)
        {
            // spin cata in the right direction
            // don't even read that command
            cata.cata_motor.spin(vex::fwd, 3.0, vex::volt);
            continue;
        }
        else
        {
            cata.cata_motor.stop();
        }

        if (this_command == CataSys::Command::Fire)
        {
            if (st.ball_in_cata)
            {
                // spin cata to fire
                cata.cata_motor.spin(vex::fwd, 5.0, vex::volt);
            }
        }
        else if (this_command == CataSys::Command::StopIntake)
        {
            cata.intake_motor.stop();
        }
        else
        {
            // until i get acess to a robot
            cata.intake_motor.spin(vex::fwd, 8.0, vex::volt);
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

void CataSys::send_command(Command next_cmd)
{
    cmd = next_cmd;
}

CataSys::State CataSys::get_state() const
{
    return state;
}

class CataSysPage : public screen::Page
{
public:
    CataSysPage(const CataSys &cs) : cs(cs) {}
    void update(bool, int, int) override {}

    void draw(vex::brain::lcd &scr, bool,
              unsigned int) override
    {
        CataSys::State state = cs.get_state();
        scr.printAt(40, 40, true, "Ball in Cata: %s", state.ball_in_cata ? "yes" : "no");
        scr.printAt(40, 60, true, "Ball in Intake: %s", state.ball_in_intake ? "yes" : "no");
        scr.printAt(40, 80, true, "Cata In Position: %s", state.cata_in_position ? "yes" : "no");
    }
    const CataSys &cs;
};

screen::Page *CataSys::Page()
{
    return new CataSysPage(*this);
}