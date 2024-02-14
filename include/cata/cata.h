#include "../core/include/utils/controls/pidff.h"
#include "cata/common.h"
#include "vex.h"
#include <../core/include/utils/state_machine.h>
#include <functional>
#include <string>

enum class CataOnlyMessage {
    DoneReloading,
    DoneFiring,
    Fire,
    Slipped,
    StartDrop,
    EnableCata,
    DisableCata,

};
enum class CataOnlyState {
    CataOff,
    WaitingForDrop,
    Firing,
    Reloading,
    ReadyToFire
};
std::string to_string(CataOnlyState s);
std::string to_string(CataOnlyMessage s);

class CataOnlySys : public StateMachine<CataOnlySys, CataOnlyState,
                                        CataOnlyMessage, 5, false> {
  public:
    friend struct Reloading;
    friend class Firing;
    friend class ReadyToFire;
    friend class WaitingForDrop;
    friend class CataOff;

    friend class CataSysPage;
    CataOnlySys(vex::pot &cata_pot, vex::optical &cata_watcher,
                vex::motor_group &cata_motor, PIDFF &cata_pid, DropMode drop);
    bool intaking_allowed();

  private:
    vex::pot &pot;
    vex::optical &cata_watcher;
    vex::motor_group &mot;
    PIDFF &pid;
};
