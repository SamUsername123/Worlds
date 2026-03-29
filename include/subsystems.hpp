#pragma once
#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "screen.hpp"




extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
extern pros::Motor intake;
extern pros::Motor outake;
extern pros::adi::DigitalOut loader;
extern pros::adi::DigitalOut wing;
extern pros::adi::DigitalOut middle_hatch;
extern pros::Distance front_distance;
extern pros::Distance back_distance;
extern pros::Distance left_distance;
extern pros::Distance right_distance;
extern pros::Optical intake_color_1;
extern pros::Optical intake_color_2;

extern int global_selected_color;

// extern pros::adi::DigitalOut park_brace;


// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');