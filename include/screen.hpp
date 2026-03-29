#include "autons.hpp"
#include "main.h"
#include <vector>
extern Better_Auton selected_auton;
extern vector<Better_Auton> autons;
extern int mode;
int main();
void start_selector(vector<Better_Auton>,Better_Auton&);
void write_selector();