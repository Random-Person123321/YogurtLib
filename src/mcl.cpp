#include "main.h"
#include "mcl.h"
#include <iomanip>


/*
TO DO:
1. Add randomness to particle spawning while getting mother particle -> Currently gives an error
2. Add reset at start weight and MCL reset for autons (Similar to reset_odom)
3. Add motion prediction
4. Add partical weight
5. Add pose calculation
6. Add weight array
7. Add likelihood calculation based off weighted particles - than move the used particles into m_particles (Mother Particle) vector
8. Add task
9. Tune partical number
10. Map feild
*/


///////////////////////////////////
//// Define variables for MCL ////
/////////////////////////////////

//Partical Definitions 
int particle_Num = 300; //Inital Particals
const int max_Particles = 500; //True maximum number of particals, used when likelihood is low
std::vector<particle_data> particles; //Where particles live
const double resample_Percent = 0.25;
double start_Weight = 2.0;

//Chance we are in a position, used for the adaptive part of AMCL
double likelihood = 75.0; //How likely we are where we think we are, 0 = No chance, 100 = Certain
const double min_Likelihood = 50.0; //Minimum likelihood before we pull all resoucres to figure out where we are

//Feild size:
const double walls = 70.5; //Tune this to ensure walls are the correct distance from (0,0)
const double x_Max = walls;
const double x_Min = -1 * walls;
const double y_Max = walls;
const double y_Min = -1 * walls;

//Obsticals


///////////////////////////////////////
//// Define variables for sensors ////
/////////////////////////////////////


//ALL distance sensor variables are robot relitive
//Ex, front sensor is ALWAYS mounted in line with the drive train and forwards
//Ensure all sensors are straight relitive to their measured axis (Up/Down is fine)
//X offset, right = positive NEEDS TO BE HORIZONTAL FROM CENTER
//Y offset, forwards = positive NEEDS TO BE HORIZONTAL FROM CENTER
//Vertical Angle, make positive no matter what, IN DEGREES (auto conversion)

//Front distance
const double fD_Yoffset = 0.0;
const double fD_Xoffset = 0.0;
const double fD_VertAngle = deg2rad(0.0);

//Back distance
const double bD_Yoffset = 0.0;
const double bD_Xoffset = 0.0;
const double bD_VertAngle = deg2rad(0.0);

//Right distance
const double rD_Yoffset = 0.0;
const double rD_Xoffset = 0.0;
const double rD_VertAngle = deg2rad(0.0);

//Left distance
const double LD_Yoffset = 0.0;
const double LD_Xoffset = 0.0;
const double LD_VertAngle = deg2rad(0.0);

//Data variables:
double data_LD = 0.0;
double data_fD= 0.0;
double data_rD = 0.0;
double data_bD = 0.0;
double mcl_Theta = pose.heading;

////////////////////////
//// Steps for MCL ////
//////////////////////

//Update amount of particals to generate (Step 1):
void update_Particle_Num(){
    double error = 100.0 - likelihood; //Invert the likelihood to make scaling easier
    particle_Num = error / min_Likelihood * max_Particles; //Calculate new partical number
    particle_Num = floor(clampd(particle_Num, 100.0, max_Particles)); //Clamp particals between 100 - max

    //Math, just for referance
    //Confidence 50-100
    //Particles 500-100
    //(maxConficence - currentConfidence) / 50 * 500
}

//Predict movement on particles (Step 2):
void predict_Movement(){

}

//Generate particals, using weight (Step 3):
void generate_Particles(){
    int mother_Particles = std::max(particle_Num * resample_Percent, (double)50); //How many particles we convert to particles
    int particles_PMP = std::floor(particle_Num / mother_Particles); //How many particles per one mother particle
    int leftover_P = particle_Num % mother_Particles;
    double x_range = 2.0; //How far off from mother particle new particles can spawn in X
    double y_range = 1.5; //How far off from mother particle new particles can spawn in Y
    double heading_range = 5.0; //Heading range from mother partical
    // particles.clear(); //Delete old particles to be able to generate new ones    
    std::vector<particle_data> new_particles; //Temporary holder for the particles

    for (int i = 0; i < mother_Particles; i++){
        double mp_x = particles[i].x; //Get x of curent mother particle
        double mp_y = particles[i].y; //Get y of current mother particle
        double mp_h = particles[i].h; //Get heading of current mother particle
        double mp_w = particles[i].weight; //Get the weight of current mother particle
        for (int j = 0; j < particles_PMP; j++){
            double rX = 0;
            double rY = 0;
            double rH = 0;
            new_particles.emplace_back(rH, rY, rH, start_Weight);
        }
    }
    particles = new_particles; //Move new particles into standard particle area

    //Calculate how many mother particles = 10%(?) of partical num, with a mimimum at 50 mother particles
    //How many particles per mother particle = particle num / mother particles, WITH LEFTOVERS
    //Assign left over particles to groups, starting with one
}

//Update sensor data (Step 4):
void update_Sensors(){
    //Get readings from each distance sensor and convert to inches, aswell as exact heading before reading
    mcl_Theta = pose.heading;
    data_LD = (LD.get_distance() / 25.4);
    data_fD= (fD.get_distance() / 25.4);
    data_rD = (rD.get_distance() / 25.4);
    data_bD = (bD.get_distance() / 25.4);

    //Use trig and sensor angle to update distance, than add offset to centre
    data_LD = (std::cos(LD_VertAngle) * data_LD) + LD_Xoffset;
    data_fD = (std::cos(fD_VertAngle) * data_fD) + fD_Yoffset;
    data_rD = (std::cos(rD_VertAngle) * data_rD) + rD_Xoffset;
    data_bD = (std::cos(bD_VertAngle) * data_bD) + bD_Yoffset;

    //Account for noise

}

//Apply weight (Step 5):
void weigh_Particles(){

}

//Caclulate Pose based off weight (Step 6):
void calc_Pose(){

}

//Re-Calculate Likelihood, using weight (Step 7):
void calc_Likelihood(){
     
}