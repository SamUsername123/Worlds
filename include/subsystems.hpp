#pragma once
#include "autons.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "screen.hpp"


class antijam_motor{
    public:
    pros::Motor* motor;
    int time_accum= 0;
    int target_speed;
    int threshold_speed;
    int wait_time;
    float max_temp;
    bool hold;
    int tries;
    int try_threshold;
    bool check;
    void jam_check();
    void move(int speed);
    void brake();
    double get_power();

    antijam_motor(){
        motor;
        target_speed =0;
        threshold_speed = 40;
        wait_time = 50;
        max_temp = 55;
        hold = false;
        tries = 0;
        try_threshold = 10;
        check = false;
    }
    antijam_motor(pros::Motor* Motor, int Threshold_speed, int Wait_time, float Max_temp,int Try_){
        motor = Motor;
        threshold_speed = Threshold_speed;
        target_speed = 0;
        wait_time = Wait_time;
        max_temp = Max_temp;
        hold = false;
        tries = 0;
        try_threshold = Try_;
        check = false;
    }
};

extern antijam_motor intake;
extern antijam_motor outake;

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
// extern pros::Motor intake;
// extern pros::Motor outake;

extern pros::adi::DigitalOut loader;
extern pros::adi::DigitalOut wing;
extern pros::adi::DigitalOut middle_hatch;
extern pros::adi::DigitalOut low_goal_arm;

extern pros::Distance front_distance;
extern pros::Distance front_distance2;
extern pros::Distance back_distance;
extern pros::Distance left_distance;
extern pros::Distance right_distance;

extern pros::Optical intake_color_1;
extern pros::Optical intake_color_2;

extern int global_selected_color;
inline double front_sensor_offset = 213.36;

// extern pros::adi::DigitalOut park_brace;

extern void start_antijam_task();

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');