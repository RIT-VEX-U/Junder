#include "cata_system.h"
#include "robot-config.h"

const double inake_enable_lower_threshold = 70.0;
const double intake_enable_upper_threshold = 110;

// const double cata_ready_lower_threshold = 85; // UNUSED - see cata_ready()
// const double cata_ready_upper_threshold = 98;

const double cata_target_charge = 98;
const double cata_target_intake = 96;


PID::pid_config_t pc = PID::pid_config_t{
    .p = 3,
    .deadband = 1,
    .on_target_time = 0.3
};
PID cata_pid(pc);

bool intake_can_be_enabled(double cata_pos)
{
    return cata_pos > inake_enable_lower_threshold && cata_pos < intake_enable_upper_threshold;
}

// bool cata_ready(double cata_pos)
// {
//     // removed lower to avoid issue where cata locks up if out of bounds
//     // return cata_pos > cata_ready_lower_threshold && cata_pos < cata_ready_upper_threshold;
//     return cata_pos < cata_ready_upper_threshold;
// }

const double intake_volts = 12.0;
int thread_func(void *void_cata)
{
    CataSys &cata = *(CataSys *)void_cata;
    cata.state = CataSys::CataState::CHARGING;

    while (true)
    {
        // read sensors
        const double cata_pos = cata.cata_pot.angle(vex::degrees);

        const bool ball_in_intake = cata.intake_watcher.isNearObject();
        const bool ball_in_cata = cata.cata_watcher.isNearObject();
        // const bool cata_in_position = cata_ready(cata_pos);

        // SYNCHRONIZE (READ)
        cata.control_mut.lock();
        CataSys::CataState cur_state = cata.state;
        bool firing_requested = cata.firing_requested;
        bool intaking_requested = cata.intaking_requested;
        CataSys::IntakeType intake_type = cata.intake_type;
        cata.control_mut.unlock();

        bool intake_cata_enabled = false;

        // Main catapult state machine
        switch(cur_state)
        {
            case CataSys::CataState::CHARGING:

                // ==== CATAPULT === 
                // Run via PID
                cata_pid.set_target(cata_target_charge);
                cata_pid.update(cata_pos);
                cata.cata_motor.spin(vex::fwd, cata_pid.get(), vex::volt);

                // ==== INTAKE ==== 
                // Disable while charging
                cata.intake_lower.stop();
                cata.intake_upper.stop();
                
                // ==== EXIT STATE ==== 
                // Ratchet engaged, we are READY
                if(cata_pid.is_on_target())
                {
                    cur_state = CataSys::CataState::READY;
                }
            break;
            case CataSys::CataState::READY:

                

                // ==== INTAKE ====
                // Run based on requested action
                // Make sure the cata is in a good position

                if (intaking_requested && intake_can_be_enabled(cata_pos) 
                    && intake_type == CataSys::IntakeType::In && !cata.cata_watcher.isNearObject())
                {
                    // Intake triball
                    cata.intake_upper.spin(vex::fwd, intake_volts, vex::volt);
                    cata.intake_lower.spin(vex::fwd, intake_volts, vex::volt);

                    cata_pid.set_target(cata_target_intake);
                    cata_pid.update(cata_pos);
                    cata.cata_motor.spin(vex::fwd, cata_pid.get(), vex::volt);
                    intake_cata_enabled = true;
                    
                } else if (intaking_requested && intake_can_be_enabled(cata_pos) 
                    && intake_type == CataSys::IntakeType::Hold)
                {
                    // Intake until ball is sensed in intake, then stop
                    if(cata.intake_watcher.isNearObject())
                    {
                        cata.intake_upper.stop(brakeType::hold);
                        cata.intake_lower.stop(brakeType::hold);
                    } else
                    {
                        cata.intake_upper.spin(vex::fwd, intake_volts, vex::volt);
                        cata.intake_lower.spin(vex::fwd, intake_volts, vex::volt);
                    }
                } else if (intaking_requested && intake_type == CataSys::IntakeType::Out)
                {
                    cata.intake_upper.spin(vex::fwd, -intake_volts, vex::volt);
                    cata.intake_lower.spin(vex::fwd, -intake_volts, vex::volt);
                } else
                {
                    cata.intake_upper.stop();
                    cata.intake_lower.stop();
                }

                // ==== CATAPULT ====
                // Disable, rely on ratchet

                if(intake_cata_enabled == false)
                {
                    cata.cata_motor.stop(brakeType::coast);
                }
                
                // When firing is requested, FIRE!
                if (firing_requested && cata.cata_watcher.isNearObject())
                {
                    cur_state = CataSys::CataState::FIRING;
                }

                // Check if position is too high, go back to charging.
                if (cata_pos > 110)
                {
                    cur_state = CataSys::CataState::CHARGING;
                }
            break;
            case CataSys::CataState::FIRING:
                // ==== INTAKE ====
                // Disable while firing
                cata.intake_upper.stop();
                cata.intake_lower.stop();

                // ==== CATAPULT ====
                cata.cata_motor.spinFor(directionType::rev, 500, timeUnits::msec);

                if(cata.cata_motor.isDone())
                {
                    cur_state = CataSys::CataState::CHARGING;
                }
            break;
        }

        if (ball_in_cata && ball_in_intake)
        {
            printf("DQed! ball in intake and catapault (or a sensor got funny)\n");
            con.rumble("-");
        }

        // SYNCHRONIZE
        cata.control_mut.lock();
        cata.state = cur_state;
        cata.control_mut.unlock();


        vexDelay(5);
    }
    return 0;
}

CataSys::CataSys(vex::optical &intake_watcher,
                 vex::pot &cata_pot,
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

CataSys::CataState CataSys::get_state() const
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
        CataSys::CataState state = cs.get_state();
        // scr.printAt(40, 40, true, "Ball in Cata: %s", state.ball_in_cata ? "yes" : "no");
        // scr.printAt(40, 60, true, "Ball in Intake: %s", state.ball_in_intake ? "yes" : "no");
        // scr.printAt(40, 80, true, "Cata In Position: %s", state.cata_in_position ? "yes" : "no");
    }
    const CataSys &cs;
};

screen::Page *CataSys::Page()
{
    return new CataSysPage(*this);
}