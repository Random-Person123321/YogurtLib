#include "yogurtlib/UI.hpp"
#include "yogurtlib/misc.hpp"
using namespace yogurt;

Drivetrain* UI::drivetrainRef = nullptr;

// Out-of-class definitions: every `static` member UI.hpp declares needs
// exactly one of these somewhere, or it has a declaration but no storage.
bool UI::do_data_updates = false;
bool UI::auto_check = true;

lv_obj_t* UI::home_scr = nullptr;
lv_obj_t* UI::logo_scr = nullptr;
lv_obj_t* UI::data_scr = nullptr;
lv_obj_t* UI::auton_scr = nullptr;
lv_obj_t* UI::error_scr = nullptr;

lv_obj_t* UI::home_bg = nullptr;
lv_obj_t* UI::data_bg = nullptr;
lv_obj_t* UI::auto_bg = nullptr;
lv_obj_t* UI::logo_bg = nullptr;

lv_obj_t* UI::logo_btn = nullptr;
lv_obj_t* UI::data_btn = nullptr;
lv_obj_t* UI::auton_btn = nullptr;

lv_obj_t* UI::logo_home_btn = nullptr;

lv_obj_t* UI::data_home_btn = nullptr;
lv_obj_t* UI::imu1_lbl = nullptr;
lv_obj_t* UI::imu2_lbl = nullptr;
lv_obj_t* UI::imu3_lbl = nullptr;
lv_obj_t* UI::pose_x_lbl = nullptr;
lv_obj_t* UI::pose_y_lbl = nullptr;
lv_obj_t* UI::heading_lbl = nullptr;
lv_obj_t* UI::calibrate_btn = nullptr;

lv_obj_t* UI::field = nullptr;
lv_obj_t* UI::selector_home_btn = nullptr;
lv_obj_t* UI::next_btn = nullptr;
lv_obj_t* UI::last_btn = nullptr;
lv_obj_t* UI::select_btn = nullptr;
lv_obj_t* UI::auton_name = nullptr;

std::vector<AutoSelection> UI::auton_list = {};
int UI::selected_auto = 0;
int UI::current_auto = 0;
int UI::imuNum = 0;

//IMU Calibration
void UI::IMUcalibrate(lv_event_t* e) {
    drivetrainRef->recalibrateIMUs();
}

void UI::screen_init(Drivetrain& dt) {
    drivetrainRef = &dt;
    // create screens FIRST
    home_scr = lv_obj_create(NULL);
    logo_scr = lv_obj_create(NULL);
    data_scr = lv_obj_create(NULL);
    auton_scr = lv_obj_create(NULL);
    error_scr = lv_obj_create(NULL);

    home_bg = lv_image_create(home_scr);
    lv_image_set_src(home_bg, "S:UI/home.bin");

    // HOME SCREEN BUTTONS
    
    data_btn = lv_button_create(home_scr);
    lv_obj_remove_style_all(data_btn);
    lv_obj_set_size(data_btn, 134, 51);
    lv_obj_align(data_btn, LV_ALIGN_TOP_LEFT, 322, 24);
    lv_obj_add_event_cb(data_btn, data_screen, LV_EVENT_CLICKED, NULL);

    logo_btn = lv_button_create(home_scr);
    lv_obj_remove_style_all(logo_btn);
    lv_obj_align(logo_btn, LV_ALIGN_TOP_LEFT, 322, 94);
    lv_obj_set_size(logo_btn, 134, 51);
    lv_obj_add_event_cb(logo_btn, logo_screen, LV_EVENT_CLICKED, NULL);

    auton_btn = lv_button_create(home_scr);
    lv_obj_remove_style_all(auton_btn);
    lv_obj_set_size(auton_btn, 134, 51);
    lv_obj_align(auton_btn, LV_ALIGN_TOP_LEFT, 322, 164);
    lv_obj_add_event_cb(auton_btn, auton_screen, LV_EVENT_CLICKED, NULL);

    // LOGO SCREEN
    logo_bg = lv_image_create(logo_scr);
    lv_image_set_src(logo_bg, "S:UI/logo.bin");
    logo_home_btn = lv_button_create(logo_scr);
    lv_obj_remove_style_all(logo_home_btn);
    lv_obj_set_size(logo_home_btn, 61, 113);
    lv_obj_align(logo_home_btn, LV_ALIGN_TOP_LEFT, 401, 48);
    lv_obj_add_event_cb(logo_home_btn, home_screen, LV_EVENT_CLICKED, NULL);

    // DATA SCREEN
    data_bg = lv_image_create(data_scr);
    lv_image_set_src(data_bg, "S:UI/data.bin");

    //Home Button
    data_home_btn = lv_button_create(data_scr);
    lv_obj_remove_style_all(data_home_btn);
    lv_obj_set_size(data_home_btn, 134, 51);
    lv_obj_align(data_home_btn, LV_ALIGN_TOP_LEFT, 322, 164);
    lv_obj_add_event_cb(data_home_btn, home_screen, LV_EVENT_CLICKED, NULL);

    //Calibrate Button
    calibrate_btn = lv_button_create(data_scr);
    lv_obj_remove_style_all(calibrate_btn);
    lv_obj_set_size(calibrate_btn, 134, 51);
    lv_obj_align(calibrate_btn, LV_ALIGN_TOP_LEFT, 24, 164);
    lv_obj_add_event_cb(calibrate_btn, IMUcalibrate, LV_EVENT_CLICKED, NULL);

    //Data screen lbls
    imu1_lbl = lv_label_create(data_scr);
    lv_obj_set_style_text_font(imu1_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(imu1_lbl, "IMU1:");
    lv_obj_align(imu1_lbl, LV_ALIGN_TOP_LEFT, 30, 110);
    lv_label_set_long_mode(imu1_lbl, LV_LABEL_LONG_CLIP);
    lv_obj_set_size(imu1_lbl, 115, 51);

    imu2_lbl = lv_label_create(data_scr);
    lv_obj_set_style_text_font(imu2_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(imu2_lbl, "IMU2:");
    lv_obj_align(imu2_lbl, LV_ALIGN_TOP_LEFT, 179, 110);
    lv_label_set_long_mode(imu2_lbl, LV_LABEL_LONG_CLIP);
    lv_obj_set_size(imu2_lbl, 115, 51);

    imu3_lbl = lv_label_create(data_scr);
    lv_obj_set_style_text_font(imu3_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(imu3_lbl, "IMU3:");
    lv_obj_align(imu3_lbl, LV_ALIGN_TOP_LEFT, 328, 110);
    lv_label_set_long_mode(imu3_lbl, LV_LABEL_LONG_CLIP);
    lv_obj_set_size(imu3_lbl, 115, 51);

    pose_x_lbl = lv_label_create(data_scr);
    lv_obj_set_style_text_font(pose_x_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(pose_x_lbl, "X:");
    lv_obj_align(pose_x_lbl, LV_ALIGN_TOP_LEFT, 30, 40);

    pose_y_lbl = lv_label_create(data_scr);
    lv_obj_set_style_text_font(pose_y_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(pose_y_lbl, "Y:");
    lv_obj_align(pose_y_lbl, LV_ALIGN_TOP_LEFT, 179, 40);

    heading_lbl = lv_label_create(data_scr);
    lv_obj_set_style_text_font(heading_lbl, &lv_font_montserrat_16, 0);
    lv_label_set_text(heading_lbl, "H:");
    lv_obj_align(heading_lbl, LV_ALIGN_TOP_LEFT, 328, 40);

    //AUTON SCREEN STUFF
    auto_bg = lv_image_create(auton_scr);
    lv_image_set_src(auto_bg, "S:UI/auton.bin");
    //Set the feild image
    field = lv_image_create(auton_scr);
    //Dont set field image cause the function and calling of the screen will handle it
    lv_obj_align(field, LV_ALIGN_TOP_LEFT, 16, 8);

    //Make the home button and add a lbl to it
    selector_home_btn = lv_button_create(auton_scr);
    lv_obj_remove_style_all(selector_home_btn);
    lv_obj_set_size(selector_home_btn, 134, 41);
    lv_obj_align(selector_home_btn, LV_ALIGN_TOP_LEFT, 322, 11);
    lv_obj_add_event_cb(selector_home_btn, home_screen, LV_EVENT_CLICKED, NULL);
    auton_name = lv_label_create (selector_home_btn);
    lv_obj_set_size(auton_name, 124, 21);
    lv_obj_set_style_text_font(auton_name, &lv_font_montserrat_16, 0);
    lv_label_set_long_mode(auton_name, LV_LABEL_LONG_DOT);
    //Dont set any text on the label, its controlled by the buttons
    lv_obj_align(auton_name, LV_ALIGN_CENTER, 0 , 1);
    lv_obj_set_style_text_align(auton_name, LV_TEXT_ALIGN_CENTER, 0);

    //Make the next button
    next_btn = lv_button_create(auton_scr);
    lv_obj_remove_style_all(next_btn);
    lv_obj_set_size(next_btn, 134, 41);
    lv_obj_align(next_btn, LV_ALIGN_TOP_LEFT, 322, 191);
    lv_obj_add_event_cb(next_btn, next_auto, LV_EVENT_CLICKED, NULL);

    //Make the last previous button
    last_btn = lv_button_create(auton_scr);
    lv_obj_remove_style_all(last_btn);
    lv_obj_set_size(last_btn, 134, 41);
    lv_obj_align(last_btn, LV_ALIGN_TOP_LEFT, 322, 128);
    lv_obj_add_event_cb(last_btn, last_auto, LV_EVENT_CLICKED, NULL);

    //Make the selector button
    select_btn = lv_imagebutton_create(auton_scr);
    lv_imgbtn_set_src(select_btn, LV_IMGBTN_STATE_RELEASED, NULL, "S:UI/select.bin", NULL);
    lv_imgbtn_set_src(select_btn, LV_IMGBTN_STATE_CHECKED_RELEASED, NULL, "S:UI/selected.bin", NULL);
    lv_obj_align(select_btn, LV_ALIGN_TOP_LEFT, 322, 70);
    lv_obj_add_event_cb(select_btn, select_auto, LV_EVENT_CLICKED, this);
    
    //Get how many IMU's are connected
    imuNum = drivetrainRef->odom.imus.count();

    //Load the home screen & start the task
    lv_screen_load(home_scr);
    lv_timer_create(update_data, 100, NULL);
}

void UI::home_screen(lv_event_t* e) {
    lv_screen_load(home_scr);
    do_data_updates = false;

}

void UI::logo_screen(lv_event_t* e) {
    lv_screen_load(logo_scr);
}

void UI::data_screen(lv_event_t* e) {
    lv_screen_load(data_scr);
    do_data_updates = true;
}

void UI::auton_screen(lv_event_t* e) {
    if(!auton_list.empty()) {
        lv_screen_load(auton_scr);
        if (auton_list[current_auto].selected) {
            lv_obj_add_state(select_btn, LV_STATE_CHECKED);
        } else {
            lv_obj_remove_state(select_btn, LV_STATE_CHECKED);
        }
        lv_label_set_text(auton_name, auton_list[current_auto].name);
        lv_image_set_src(field, auton_list[current_auto].file);
    }
}

void UI::select_auto(lv_event_t* e) {
    auton_list[selected_auto].selected = false; //Turn off old auton selected button
    selected_auto = current_auto; //Make the current auton selected
    auton_list[selected_auto].selected = true; //Update the vector so the pages are nice :)
    lv_obj_add_state(select_btn, LV_STATE_CHECKED);
}

void UI::next_auto(lv_event_t* e) {
    current_auto += 1;
    if (current_auto >= auton_list.size()) {
        current_auto = 0;
    }
    sync_Auton_Selector();
}

void UI::last_auto(lv_event_t* e) {
    current_auto -= 1;
    if (current_auto < 0) {
        current_auto = auton_list.size() -1;
    }
    sync_Auton_Selector();
}

void UI::sync_Auton_Selector() {
    if (auton_list[current_auto].selected) {
        lv_imgbtn_set_state(select_btn, LV_IMGBTN_STATE_CHECKED_RELEASED);
    } else {
        lv_imgbtn_set_state(select_btn, LV_IMGBTN_STATE_RELEASED);
    }
    lv_label_set_text(auton_name, auton_list[current_auto].name);
    lv_image_set_src(field, auton_list[current_auto].file);
}


void UI::update_data(lv_timer_t* t) {
    if (do_data_updates) {
        char buf[20];
        snprintf(buf, sizeof(buf), "X: %.2f", drivetrainRef->getPose().x);
        lv_label_set_text(pose_x_lbl, buf);
        
        snprintf(buf, sizeof(buf), "Y: %.2f", drivetrainRef->getPose().y);
        lv_label_set_text(pose_y_lbl, buf);
        
        snprintf(buf, sizeof(buf), "H: %.2f", wrapDeg(drivetrainRef->getPose().theta));
        lv_label_set_text(heading_lbl, buf);
        
        if (imuNum >= 1) {
            const char* state = !drivetrainRef->odom.imus.isValid(0) ? "OFF" : (drivetrainRef->odom.imus.isBenched(0) ? "BENCH" : "OK");
            snprintf(buf, sizeof(buf), "1: %.2f, %s", drivetrainRef->odom.imus.getHeading(0), state);
            lv_label_set_text(imu1_lbl, buf);
        }
        
        if (imuNum >= 2) {
            const char* state = !drivetrainRef->odom.imus.isValid(1) ? "OFF" : (drivetrainRef->odom.imus.isBenched(1) ? "BENCH" : "OK");
            snprintf(buf, sizeof(buf), "2: %.2f, %s", drivetrainRef->odom.imus.getHeading(1), state);
            lv_label_set_text(imu2_lbl, buf);
        }
        
        if (imuNum >= 3) {
            const char* state = !drivetrainRef->odom.imus.isValid(2) ? "OFF" : (drivetrainRef->odom.imus.isBenched(2) ? "BENCH" : "OK");
            snprintf(buf, sizeof(buf), "3: %.2f, %s ", drivetrainRef->odom.imus.getHeading(2), state);
            lv_label_set_text(imu3_lbl, buf);
        }


    }
    if (auto_check) {
        if(pros::competition::is_autonomous()) {
            UI::logo_screen(nullptr);
            auto_check = false;
        }
    }
}