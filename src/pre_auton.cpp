#include "main.h"

void SAWP_INIT() {
    // SAWP D Resets
    resetOdometry(16.8, -47.0, 0.0);
    for (int i = 0; i < 3; i++) {
        distanceReset(fD, 70.5, 9.0, true, true);
        distanceReset(rD, -70.5, 3.5, false, false);
    }
}