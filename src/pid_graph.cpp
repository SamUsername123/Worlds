#include <cstdio>
#include "main.h"
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/PID.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include <vector>

int main(){}

void start_graph(double target_x,double target_y,double target_theta, PID& selected_pid ){
    double last_error;
    int counter = 0;
    while (true){
        double dx = target_x - chassis.odom_x_get();
        double dy = target_y - chassis.odom_y_get();
        double error_xy = sqrt(dx*dx + dy*dy);
        double error_theta = target_theta - chassis.odom_theta_get();
        printf(">XY_Error:%.2f\n",error_xy);
        printf(">Theta_Error:%.2f\n",error_theta);
        if (0.95*last_error <= error_xy && error_xy<= 1.05*last_error){
            counter++;
        } else {
            counter = 0;
        }
        if (counter >= 10){
            break;
        }
        last_error = error_xy;
        pros::delay(25);
    }
}
