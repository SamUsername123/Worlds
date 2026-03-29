#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/device.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

int main(){}

void main_controller_task(){
    // master.clear();
    int time_accum;
    int x = 0;
    int last_x = 0;
    while (true){
        master.print(0, 2, "%s            ", selected_auton.auton_data.Name);
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){x++;}
        else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){x--;}
        if ( x != last_x){
            // pros::c::controller_clear(pros::E_CONTROLLER_MASTER);
            if (x > autons.size()-1){
                x = 0;
            } else if (x<0){
                x = autons.size()-1;
            }
            selected_auton = autons[x];
            write_selector();
            
        }
        last_x = x;
        pros::delay(50);
    }
}