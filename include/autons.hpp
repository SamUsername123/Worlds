#pragma once
#include "EZ-Template/auton.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "pros/distance.hpp"
#include "pros/optical.hpp"
#include <vector>
#include <string>
void default_constants();
okapi::QLength get_average_distance(int,pros::Distance,int);
okapi::QLength distance_to_wall(pros::Distance,okapi::QLength);
okapi::QAngle get_front_angle_offset();
void matchload(int);
void right_7();
void left_7();
void left_split();
void right_super_4();
void skills_old();
void skills();
void angle_test();
void left_super_4();
void solo_win_point();
void solo_win_point2();
void solo_win_point3();
void drive_example();
void turn_example();
void drive_and_turn();
void wait_until_change_speed();
void swing_example();
void motion_chaining();
void combining_movements();
void interfered_example();
void odom_drive_example();
void odom_pure_pursuit_example();
void odom_pure_pursuit_wait_until_example();
void odom_boomerang_example();
void odom_boomerang_injected_pure_pursuit_example();
void measure_offsets();
void odom_drive_test();
void odom_boomerang_test();
void skills_minor_skill();
void turn_test();   
void odom_long_drive();
void none();
void test_auto();
void matchload_test();
std::vector<double> distance_values_(std::string);
okapi::QLength distance_to_wall(pros::Distance distance_sensor, okapi::QLength target);

class Better_Auton {
public:
    Better_Auton() : auton_data("", nullptr), distances({}) {}
    Better_Auton(ez::Auton a, std::vector<double> d) 
        : auton_data(a), distances(d) {}
    ez::Auton auton_data;
    std::vector<double> distances;
};
