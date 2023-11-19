#include "cata_system.h"

int thread_func(void *void_cata)
{
    CataSys &cata = *(CataSys *)void_cata;
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

        // SYNCHRONIZE
        cata.control_mut.lock();
        cata.state = st;
        bool firing_requested = cata.firing_requested;
        bool intaking_requested = cata.intaking_requested;
        CataSys::IntakeType intake_type = cata.intake_type;
        cata.control_mut.unlock();

        // decide accordingly
        if (!st.cata_in_position)
        {
            // spin cata in the right direction
            cata.cata_motor.spin(vex::fwd, 3.0, vex::volt);
            continue;
        }
        else
        {
            cata.cata_motor.stop();
        }

        // fire if we should be firing
        if (firing_requested && st.ball_in_cata)
        {
            cata.cata_motor.spin(vex::fwd, 3.0, vex::volt);
            cata.intake_motor.stop();
        }
        else
        {
            cata.cata_motor.stop();
        }

        if (intaking_requested)
        {
            // intake if we should be intaking and if we won't jam cata
            if (st.cata_in_position && intake_type == CataSys::IntakeType::In) // DOESNT HANDLE INTAKE HOLD YET, JUST INTAKE ALL THE WAY
            {
                cata.intake_motor.spin(vex::fwd, 4.0, vex::volt);
            }
            else if (intaking_requested && intake_type == CataSys::IntakeType::Out)
            {
                cata.intake_motor.spin(vex::fwd, -4.0, vex::volt);
            }
        }
        else
        {
            cata.intake_motor.stop();
        }

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
                                                   intake_motor(intake_motor),
                                                   firing_requested(false),
                                                   intaking_requested(false),
                                                   intake_type(CataSys::IntakeType::In)
{
    runner = vex::task(thread_func, (void *)this);
}

void CataSys::send_command(Command next_cmd)
{
    control_mut.lock();
    switch (next_cmd)
    {
    case CataSys::Command::StartFiring:
        firing_requested = true;
        break;
    case CataSys::Command::StopFiring:
        firing_requested = false;
    case CataSys::Command::IntakeIn:
    case CataSys::Command::IntakeHold: // not handled
        intaking_requested = true;
        intake_type = CataSys::IntakeType::In;
        break;
    case CataSys::Command::IntakeOut:
        intaking_requested = true;
        intake_type = CataSys::IntakeType::Out;
    case CataSys::Command::StopIntake:
        intaking_requested = false;
        break;
    default:
        break;
    }
    control_mut.unlock();
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