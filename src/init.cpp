#include "init.h"
#include "main.h"
#include "brain_photo.h"
#include "liblvgl/lvgl.h"
#include "pros/apix.h"
#include "liblvgl/lv_api_map_v8.h"

lv_obj_t* image;

void init_brain_image() {
    LV_IMG_DECLARE(I210Comet);
    image = lv_img_create(lv_scr_act());
    lv_img_set_src(image, &I210Comet);
    lv_obj_set_size(image, 480, 240);
    lv_obj_align(image, LV_ALIGN_CENTER, 0, 0);
}

void start_tasks() {
    pros::Task UpdateOdometry(updateOdometry);
    pros::Task LcdPrint(terminalDebug);
}

