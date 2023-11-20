#include "cata_system.h"
#include "robot-config.h"

const double cata_lower_threshold = 325.0;
const double cata_upper_threshold = 345.0;

bool intake_can_be_enabled(double cata_pos)
{
    return cata_pos > cata_lower_threshold && cata_pos < cata_upper_threshold;
}

bool cata_ready(double cata_pos)
{
    return cata_pos > 315 && cata_pos < 325;
}


const double intake_volts = 12.0;
int thread_func(void *void_cata)
{
    CataSys &cata = *(CataSys *)void_cata;
    cata.state = {
        .ball_in_intake = false,
        .ball_in_cata = false,
        .cata_in_position = false};

    while (true)
    {
        // read sensors
        const double cata_pos = cata.cata_pot.angle(vex::degrees);

        const CataSys::State st = {
            .ball_in_intake = cata.intake_watcher.isNearObject(),
            .ball_in_cata = cata.cata_watcher.isNearObject(),
            .cata_in_position = cata_ready(cata_pos)};

        if (st.ball_in_cata && st.ball_in_intake)
        {
            printf("DQed! ball in intake and catapault (or a sensor got funny)\n");
            con.rumble("-");
        }

        // SYNCHRONIZE
        cata.control_mut.lock();
        cata.state = st;
        bool firing_requested = cata.firing_requested;
        bool intaking_requested = cata.intaking_requested;
        CataSys::IntakeType intake_type = cata.intake_type;
        cata.control_mut.unlock();

        // decide
        if (intaking_requested)
        {
            // intake if we should be intaking and if we won't jam cata
            if (intake_can_be_enabled(cata_pos) && intake_type == CataSys::IntakeType::In)
            {
                cata.intake_upper.spin(vex::fwd, intake_volts, vex::volt);
                cata.intake_lower.spin(vex::fwd, intake_volts, vex::volt);
            }
            else if (!intake_can_be_enabled(cata_pos) && intake_type == CataSys::IntakeType::In)
            {
                cata.intake_lower.stop(vex::brakeType::coast);
                cata.intake_upper.stop(vex::brakeType::coast);
            }
            else if (intake_type == CataSys::IntakeType::Out)
            {
                cata.intake_upper.spin(vex::fwd, -intake_volts, vex::volt);
                cata.intake_lower.spin(vex::fwd, -intake_volts, vex::volt);
            }
            else if (intake_type == CataSys::IntakeType::Hold)
            {
                cata.intake_upper.spin(vex::fwd, -intake_volts, vex::volt);
                cata.intake_lower.spin(vex::fwd, intake_volts, vex::volt);
            }
        }
        else
        {
            cata.intake_lower.stop(vex::brakeType::coast);
            cata.intake_upper.stop(vex::brakeType::coast);
        }

        // fire if we should be firing
        if (!st.cata_in_position)
        {
            // cata.cata_motor.spin(vex::fwd, 5.0, vex::volt);
        }
        else if (firing_requested && st.ball_in_cata)
        {
            // cata.cata_motor.spin(vex::fwd, 5.0, vex::volt);
            cata.intake_upper.stop(vex::brakeType::coast);
            cata.intake_lower.stop(vex::brakeType::coast);
        }
        else
        {
            cata.cata_motor.stop(vex::brakeType::coast);
        }

        vexDelay(20);
    }
    return 0;
}

CataSys::CataSys(vex::optical &intake_watcher,
                 vex::rotation &cata_pot,
                 vex::optical &cata_watcher,
                 vex::motor_group &cata_motor,
                 vex::motor &intake_upper,
                 vex::motor &intake_lower) : intake_watcher(intake_watcher),
                                             cata_pot(cata_pot),
                                             cata_watcher(cata_watcher),
                                             cata_motor(cata_motor),
                                             intake_upper(intake_upper),
                                             intake_lower(intake_lower),
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
        break;
    case CataSys::Command::IntakeIn:
        intaking_requested = true;
        intake_type = CataSys::IntakeType::In;
        break;
    case CataSys::Command::IntakeOut:
        intaking_requested = true;
        intake_type = CataSys::IntakeType::Out;
        break;
    case CataSys::Command::IntakeHold: // not handled
        intaking_requested = true;
        intake_type = CataSys::IntakeType::Hold;
        break;
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