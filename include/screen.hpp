#include "autons.hpp"
#include "main.h"
#include "pros/motors.hpp"
#include <vector>
extern Better_Auton selected_auton;
extern vector<Better_Auton> autons;
extern vector<pros::Motor> drivetrain_motors;
extern int mode;
extern int selected_animation;
int main();
void start_selector(vector<Better_Auton>,Better_Auton&);
void write_selector();