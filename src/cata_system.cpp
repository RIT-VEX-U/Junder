#include "cata_system.h"
#include "robot-config.h"

// const double inake_enable_lower_threshold = 70.0;
// const double intake_enable_upper_threshold = 120;
const double inake_enable_lower_threshold = 150;
const double intake_enable_upper_threshold = 200;

const double intake_upper_volt = 12;
const double intake_upper_volt_hold = 6;
const double intake_lower_volt = 10.0;
const double intake_sensor_dist_mm = 150;

// const double cata_ready_lower_threshold = 85; // UNUSED - see cata_ready()
// const double cata_ready_upper_threshold = 98;

// const double cata_target_charge = 96.5;
// const double cata_target_intake = 96.5;
const double cata_target_charge = 177;
const double cata_target_intake = 177;
// const double cata_target_charge = 105;
// const double cata_target_intake = 105;



PID::pid_config_t pc = PID::pid_config_t{
    .p = 1,
    // .i = 2,
    .deadband = 2,
    .on_target_time = 0.3
};

FeedForward::ff_config_t ffc = 
{
    .kG = -2
};

PIDFF cata_pid(pc, ffc);

bool intake_can_be_enabled(double cata_pos)
{
    return (cata_pos == 0.0) ||  (cata_pos > inake_enable_lower_threshold && cata_pos < intake_enable_upper_threshold);
}

// bool cata_ready(double cata_pos)
// {
//     // removed lower to avoid issue where cata locks up if out of bounds
//     // return cata_pos > cata_ready_lower_threshold && cata_pos < cata_ready_upper_threshold;
//     return cata_pos < cata_ready_upper_threshold;
// }

int thread_func(void *void_cata)
{
    CataSys &cata = *(CataSys *)void_cata;
    cata.state = CataSys::CataState::CHARGING;

    cata_pid.set_limits(-12.0, 0);

    vex::timer intake_tmr;

    while (true)
    {
        // read sensors
        const double cata_pos = cata.cata_pot.angle(vex::degrees);

        if (cata.intake_watcher.objectDistance(distanceUnits::mm) < intake_sensor_dist_mm)
            intake_tmr.reset();

        const bool ball_in_intake = 
            cata.intake_watcher.objectDistance(distanceUnits::mm) < intake_sensor_dist_mm 
            || intake_tmr.time(timeUnits::msec) < 500;

        const bool ball_in_cata = cata.cata_watcher.isNearObject();

        // SYNCHRONIZE (READ)
        cata.control_mut.lock();
        CataSys::CataState cur_state = cata.state;
        bool firing_requested = cata.firing_requested;
        bool intaking_requested = cata.intaking_requested;
        bool matchload_requested = cata.matchload_requested;
        CataSys::IntakeType intake_type = cata.intake_type;
        cata.control_mut.unlock();

        bool intake_cata_enabled = false;
        double cata_pid_out = 0;

        // Main Intake State Machine
        switch(cur_state)
        {
            case CataSys::CataState::FIRING:
                cata.intake_upper.stop();
                cata.intake_lower.stop();
            break;
            case CataSys::CataState::CHARGING:
            case CataSys::CataState::READY:
                // Charging & Ready: Base off of cata position
                // ==== INTAKE ====
                // Run based on requested action
                // Make sure the cata is in a good position

                if (intaking_requested && intake_can_be_enabled(cata_pos) 
                    && intake_type == CataSys::IntakeType::In && !ball_in_cata)
                {
                    // Intake triball
                    cata.intake_upper.spin(vex::fwd, intake_upper_volt, vex::volt);
                    cata.intake_lower.spin(vex::fwd, intake_lower_volt, vex::volt);
                    
                } else if (intaking_requested && intake_can_be_enabled(cata_pos) 
                    && intake_type == CataSys::IntakeType::Hold)
                {
                    // Intake until ball is sensed in intake, then stop
                    // timer to make sure it doesn't go straight through
                    if(ball_in_intake)
                    {
                        cata.intake_upper.stop(brakeType::hold);
                        cata.intake_lower.stop(brakeType::hold);
                    } else
                    {
                        cata.intake_upper.spin(vex::fwd, intake_upper_volt_hold, vex::volt);
                        cata.intake_lower.spin(vex::fwd, intake_lower_volt, vex::volt);
                    }
                } else if (intaking_requested && intake_type == CataSys::IntakeType::Out)
                {
                    cata.intake_upper.spin(vex::fwd, -intake_upper_volt, vex::volt);
                    cata.intake_lower.spin(vex::fwd, -intake_lower_volt, vex::volt);
                } else
                {
                    cata.intake_upper.stop();
                    cata.intake_lower.stop();
                }

                // Reset requests from AutoCommands
                if ((intake_type == CataSys::IntakeType::Hold && ball_in_intake) ||
                    (intake_type == CataSys::IntakeType::In && ball_in_cata))
                {
                    // intaking_requested = false;
                }
        }

        // Main catapult state machine
        switch(cur_state)
        {
            case CataSys::CataState::CHARGING:

                // ==== CATAPULT === 
                // Run via PID
                cata_pid.set_target(cata_target_charge);
                cata_pid_out = cata_pid.update(cata_pos);
                cata.cata_motor.spin(vex::fwd, cata_pid_out, vex::volt);
                
                // ==== EXIT STATE ==== 
                // Ratchet engaged, we are READY
                if(cata_pid.is_on_target() || cata_pos < cata_target_charge - pc.deadband)
                {
                    cur_state = CataSys::CataState::READY;
                }
            break;
            case CataSys::CataState::READY:

                // ==== CATAPULT ====
                // Disable, rely on ratchet

                if(intake_cata_enabled == false)
                {
                    // cata.cata_motor.stop(brakeType::coast);

                    // REMOVE IF RATCHETING
                    cata_pid.set_target(cata_target_charge);
                    cata_pid_out = cata_pid.update(cata_pos);
                    cata.cata_motor.spin(vex::fwd, cata_pid_out, vex::volt);
                }
                
                // When firing is requested, FIRE!
                if (firing_requested && cata.cata_watcher.isNearObject())
                {
                    cur_state = CataSys::CataState::FIRING;
                }

                // Check if position is too high, go back to charging.
                if (cata_pos > cata_target_charge + 30)
                {
                    cur_state = CataSys::CataState::CHARGING;
                }
            break;
            case CataSys::CataState::FIRING:

                // ==== CATAPULT ====
                cata.cata_motor.spinFor(directionType::rev, 500, timeUnits::msec);
                cata_pid.reset(); //reset integral

                if(cata.cata_motor.isDone())
                {
                    cur_state = CataSys::CataState::CHARGING;
                    firing_requested = false;
                }
            break;
        }

        

        printf("in cata? %d, in intake? %d, intake_dist: %f, pot: %.2f, pid: %.2f\n",
            ball_in_cata, ball_in_intake, intake_watcher.objectDistance(distanceUnits::mm), cata_pos, cata_pid.get());

        if (cata.cata_watcher.isNearObject() && cata.intake_watcher.objectDistance(distanceUnits::mm) < 150)
        {
            printf("DQed! ball in intake and catapault (or a sensor got funny)\n");
            con.rumble("-");
        }

        // SYNCHRONIZE
        cata.control_mut.lock();
        cata.state = cur_state;
        cata.firing_requested = firing_requested;
        cata.intaking_requested = intaking_requested;
        cata.control_mut.unlock();


        vexDelay(5);
    }
    return 0;
}

CataSys::CataSys(vex::distance &intake_watcher,
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
    case CataSys::Command::StartMatchLoad:
        matchload_requested = true;
        break;
    case CataSys::Command::StopMatchLoad:
        matchload_requested = false;
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

AutoCommand *CataSys::Fire()
{
    FunctionCommand *retval = new FunctionCommand([&](){
        control_mut.lock();
        firing_requested = true;
        control_mut.unlock();        
        return true;
    });

    return retval;
}

AutoCommand *CataSys::IntakeFully()
{
    FunctionCommand *retval = new FunctionCommand([&](){
        control_mut.lock();
        intaking_requested = true;
        intake_type = IntakeType::In;
        control_mut.unlock();
        return true;
    });

    return retval;
}