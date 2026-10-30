#pragma once
#include <vector>
#include "liblvgl/lvgl.h"
#include "liblvgl/lv_api_map_v8.h"
#include "yogurtlib/dt/dt.hpp"
#include "yogurtlib/pose.hpp"

namespace yogurt {class Drivetrain;} // Declaration, definition in dt.hpp

namespace yogurt {

struct AutoSelection{
    const char* name;
    const char* file;
    bool selected = false;
};

class UI {

    static Drivetrain* drivetrainRef;

    static bool do_data_updates;
    static bool error_screen_loaded;
    static int current_auto;
    static bool auto_check;
    
    // DECLARE SCREENS 
    static lv_obj_t* home_scr;
    static lv_obj_t* logo_scr;
    static lv_obj_t* data_scr;
    static lv_obj_t* auton_scr;
    static lv_obj_t* error_scr;
    static lv_obj_t* bg;

    // Home screen buttons 
    static lv_obj_t* logo_btn;
    static lv_obj_t* data_btn;
    static lv_obj_t* auton_btn;

    // Logo screen buttons
    static lv_obj_t* logo_home_btn;

    // Data screen buttons + windows
    static lv_obj_t* data_home_btn;
    static lv_obj_t* imu1_lbl;
    static lv_obj_t* imu2_lbl;
    static lv_obj_t* imu3_lbl;
    static lv_obj_t* pose_x_lbl;
    static lv_obj_t* pose_y_lbl;
    static lv_obj_t* heading_lbl;
    static lv_obj_t* calibrate_btn;

    //Auton Selector stuff
    static lv_obj_t* field;
    static lv_obj_t* selector_home_btn;
    static lv_obj_t* next_btn;
    static lv_obj_t* last_btn;
    static lv_obj_t* select_btn;
    static lv_obj_t* auton_name;    

    //This loads screens for a LVGL event when a button is pressed
    static void home_screen(lv_event_t* e);
    static void data_screen(lv_event_t* e);
    static void auton_screen(lv_event_t* e);
    static void logo_screen(lv_event_t* e);
    //Auton selector logic functions
    static void select_auto(lv_event_t* e);
    static void next_auto(lv_event_t* e);
    static void last_auto(lv_event_t* e);
    //LVGL task to update the IMU data on the data screen
    static void update_data(lv_timer_t* t);
    static void IMUcalibrate(lv_event_t* e);



public:

    /**
     * @brief Initializes the screen, call it in init
     * 
     *  ### Example Code:
     * 
     * ```
     * void initialize() {
     *   uiobject.screen_init(drivetrain);
     * } 
     * ```
     */
    void screen_init(Drivetrain& dt);

    /**
     * @brief The list of all autons, each auton requires:
     *
     * - A name to be displayed (Eg. "Blue")
     *
     * - A file location on the SD card (Eg. "S:/autons/blue.bin")
     *
     * - If it should be selected by default for a program slot (True/False)
     * 
     * ### Example Code:
     *
     * ```uiobject.auton_list.push_back( {"Blue", "S:/autons/blue.bin", false} );```
    */
    static std::vector <AutoSelection> auton_list;
    
    /**
     * @brief The selected auto from the auton selector list, use this in autonomus to determine what auto to run
    */
    static int selected_auto;

};
}//namespace yogurt
