#pragma once

// namespace pros { class Distance; }
#include "main.h"

void update_Particle_Num();

void generate_Particles();

void predict_Movement();

void update_Sensors();

void weigh_Particles();

void calc_Pose();

void calc_Likelyhood();