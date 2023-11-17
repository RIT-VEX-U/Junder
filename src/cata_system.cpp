#include "cata_system.h"


int thread_func(void * void_cata){
    CataSys &cata = *(CataSys*)void_cata;

    while (true){


    }
    return 0;
}

CataSys::CataSys(vex::optical &intake_watcher, vex::encoder &cata_enc, vex::optical &cata_watcher): intake_watcher(intake_watcher), cata_enc(cata_enc), cata_watcher(cata_watcher){
    runner = vex::task(thread_func, this);
}