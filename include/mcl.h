#ifndef MCL_H_
#define MCL_H_

// namespace pros { class Distance; }
#include "api.h"

void update_Particle_Num();

void generate_Particles();

void predict_Movement();

void update_Sensors();

void weigh_Particles();

void calc_Pose();

void calc_Likelyhood();

struct particle_data {
    double x, y, h, weight;
};

#endif