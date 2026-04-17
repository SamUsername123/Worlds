#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/adi.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

void antijam_motor::move(int speed){
    if (!this->hold){
        this->motor->move(speed);
    }
    this->target_speed = speed;
}

void antijam_motor::brake(){
    this->motor->brake();
    this->target_speed = 0;
}

double antijam_motor::get_power(){
    return this->motor->get_power();
}

void antijam_motor::jam_check(){
    double temp = this->motor->get_temperature();
    double speed = this->motor->get_actual_velocity();
    if (temp> this->max_temp){return;}
    if (this->target_speed > 0 && speed < this->threshold_speed){
        this->tries ++;
        if (this->tries >= this->try_threshold){
            this->hold = true;
            this->motor->move(-this->target_speed);
            pros::delay(wait_time);
            this->motor->move(this->target_speed);
            this->hold = false;
            this->tries = 0;
        }
    } else {
        this->tries = 0;
    }
}

void start_antijam_task(){
    while (true){
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2) || master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            pros::delay(250);
        }
        if (intake.check){intake.jam_check();}
        if (outake.check){outake.jam_check();}
        pros::delay(10);
    }
}