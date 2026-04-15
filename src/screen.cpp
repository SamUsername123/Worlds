#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "main.h"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include "EZ-Template/auton.hpp"
#include "pros/colors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "subsystems.hpp"
#include "subsystems.hpp"
#include "EZ-Template/drive/drive.hpp"
#include "controller.hpp"

int main(){}

struct Auton_btn {
    Better_Auton auto_data;
    int x1,y1,x2,y2;
    pros::text_format_e_t text_size;
};

vector<Auton_btn> make_buttons(vector<Better_Auton> autons){
    vector<Auton_btn> auton_buttons;
    int auto_index = 0;
    int per_row = ceil(autons.size() / 2.00 );
    int ypadding = 20;
    int xpadding = 210;
    int width = 120;
    int height = (220 - (ypadding*per_row))/per_row;
    for (const Better_Auton& auton : autons){
        int col = auto_index / per_row;
        int row = auto_index % per_row;
        int x = 20 + (col*(width+ xpadding));
        int y = ypadding + (row*(height + ypadding));
        int x2 = x + width;
        int y2 = y + height;
        auton_buttons.push_back({auton,x,y,x2,y2,pros::E_TEXT_MEDIUM});
        auto_index++;
    }
    return auton_buttons;
}



static constexpr int W = 480;
static constexpr int H = 272;
static constexpr int FRAME_PIXELS = W *H;
static constexpr int FRAME_BYTES = FRAME_PIXELS * 4;

static uint32_t frame_buffer[FRAME_PIXELS];

bool draw_frame(int index){
    char path[64];
    switch (selected_animation){
        case 1:
        snprintf(path, sizeof(path), "/usd/shrek/frame_%04d.bin",index);
        if (index != 0 && index % 3 == 0){
            return true;
        }
        break;
        case 2:
        snprintf(path, sizeof(path), "/usd/rick/frame_%04d.bin",index);
        if (index != 0 && index % 3 == 0){
        } else {
            return true;
        }
        break;
        case 3:
        snprintf(path, sizeof(path), "/usd/bee/frame_%04d.bin",index);
        break;
        case 4:
        snprintf(path, sizeof(path), "/usd/minecraft/frame_%04d.bin",index);
        if (index != 0 && index % 8 == 0){
        } else {
            return true;
        }
        break;
    }
    FILE *f = fopen(path, "rb");
    if (!f){
        return false;
    }
    size_t read = fread(frame_buffer, 4, FRAME_PIXELS, f);
    fclose(f);
    if ((int)read != FRAME_PIXELS){
        return false;
    }
    pros::screen::copy_area(0, 0, W-1, H-1, frame_buffer, W);
    return true;
}


void write_selector(){
    FILE * f = fopen("/usd/auton.txt", "w");
    if (f){
        fprintf(f, "%s",selected_auton.auton_data.Name.c_str());
        fflush(f);
        fclose(f);
    }
}

void write_color(){
    FILE * f = fopen("/usd/color.txt", "w");
    if (f){
        switch (global_selected_color){
            case 1:
            fprintf(f, "blue");
            break;
            case 2:
            fprintf(f, "red");
            break;
        }
        fflush(f);
        fclose(f);
    }
}

std::string check_overheat(){
    double bad_temp = 50.f;
    for (auto motor: drivetrain_motors){
        double motor_temp = motor.get_temperature();
        if (motor_temp >= bad_temp){
            return "drive";
        }
    }
    if (intake.get_temperature() >= bad_temp){
        return "intake";
    } else if (outake.get_temperature() >= bad_temp){
        return "outake";
    } else{
        return "";
    }
}

bool check_alignment(){
    double front_target = selected_auton.distances[0];
    double left_target = selected_auton.distances[1];
    double back_target = selected_auton.distances[2];
    double right_target = selected_auton.distances[3];
    double active_y_target;
    double active_x_target;
    double active_x;
    double active_y;
    if (front_target == 0 && left_target == 0 && right_target ==0 && back_target == 0){
        return true;
    }
    if (front_target !=0){
        double front_distance_value = front_distance.get_distance();
        pros::screen::print(pros::E_TEXT_MEDIUM,250,100,"Front Target: %.0f",front_target);
        pros::screen::print(pros::E_TEXT_MEDIUM,250,120,"Front Actual: %.0f",front_distance_value);
        active_y_target = front_target;
        active_y = front_distance_value;
    } else {
        double back_distance_value = back_distance.get_distance();
        pros::screen::print(pros::E_TEXT_MEDIUM,250,100,"Back Target: %.0f",back_target);
        pros::screen::print(pros::E_TEXT_MEDIUM,250,120,"Back Actual: %.0f",back_distance_value);
        active_y_target = back_target;
        active_y = back_distance_value;
    }
    if (left_target !=0){
        double left_distance_value = left_distance.get_distance();
        pros::screen::print(pros::E_TEXT_MEDIUM,250,145,"Left Target: %.0f",left_target);
        pros::screen::print(pros::E_TEXT_MEDIUM,250,165,"Left Actual: %.0f",left_distance_value);
        active_x_target = left_target;
        active_x = left_distance_value;
    } else {
        double right_distance_value = right_distance.get_distance();
        pros::screen::print(pros::E_TEXT_MEDIUM,250,145,"Right Target: %.0f",right_target);
        pros::screen::print(pros::E_TEXT_MEDIUM,250,165,"Right Actual: %.0f",right_distance_value);
        active_x_target = right_target;
        active_x = right_distance_value;
    }
    if (0.95*active_x_target <= active_x && active_x <= 1.05*active_x_target && 0.95*active_y_target <= active_y && active_y <= 1.05*active_y_target){
        return true;
    }
    else {
        return false;
    }

}

void start_selector(vector<Better_Auton> autons,Better_Auton& selected_auton){
    pros::delay(2500);
    bool color_name = false;
    enum  pros::Color selected_color;
    switch (global_selected_color){
        default:
        selected_color = pros::Color::blue;
        break;
        case 2:
        selected_color = pros::Color::red;
        break;
    }
    enum  pros::Color last_color;
    pros::screen::set_eraser(pros::Color::black);
    pros::screen::set_pen(pros::Color::white);
    pros::screen::erase();
    string last_selected_name = selected_auton.auton_data.Name;
    vector<Auton_btn> auton_buttons = make_buttons(autons);
    int last_mode = mode;
    int presses = 0;
    int index = 16;
    int frames = 76;
    int delay_time=40;
    int last_animation = 1;
    while (true) {
        
        pros::screen_touch_status_s_t status = pros::screen::touch_status();
        if (mode != last_mode){
            delay_time =40;
            pros::screen::set_eraser(pros::Color::black);
            pros::screen::erase();
        }
        last_mode = mode;
         if (selected_animation != last_animation) {
        switch (selected_animation){
            case 1:
                index = 16;
                frames = 76;
                break;
            case 2:
                index = 0;
                frames = 158;
                break;
            case 3:
                index = 0;
                frames = 469;
                break;
            case 4:
                index = 0;
                frames = 891;
                break;
        }
        last_animation = selected_animation;
    }
        if (mode ==1){
            std::string overheated = check_overheat();
            pros::screen::set_eraser(selected_color);
            if (selected_auton.auton_data.Name != last_selected_name or last_color != selected_color){
                pros::screen::set_eraser(pros::Color::black);
                pros::screen::erase();
                master.print(0, 0, "%s     ", selected_auton.auton_data.Name);
            }

            for (const Auton_btn& auton : auton_buttons){
                pros::screen::set_pen(selected_color);
                if (auton.auto_data.auton_data.Name == selected_auton.auton_data.Name){
                    pros::screen::fill_rect(auton.x1,auton.y1,auton.x2,auton.y2);
                    pros::screen::set_eraser(selected_color);
                } else {
                    pros::screen::draw_rect(auton.x1,auton.y1,auton.x2,auton.y2);
                    pros::screen::set_eraser(pros::Color::black);
                }
                pros::screen::set_pen(pros::Color::white);
                pros::screen::print(pros::E_TEXT_MEDIUM,(auton.x1+10),(auton.y1+10),"%s",auton.auto_data.auton_data.Name);   
            }
            // pros::screen::draw_rect(180,65,320,145);
            pros::screen::set_eraser(pros::Color::black);
            pros::screen::print(pros::E_TEXT_LARGE,180,20,"11005B");
            pros::screen::print(pros::E_TEXT_SMALL,200,220,"Battery: %.0f %",pros::battery::get_capacity());
            pros::screen::set_pen(pros::Color::blue);
            if (selected_color == pros::Color::blue){
                pros::screen::set_eraser(pros::Color::blue);
                pros::screen::fill_rect(180,155,244,200);
            } else {
                pros::screen::set_eraser(pros::Color::black);
                pros::screen::draw_rect(180,155,244,200);
            }
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,190,170,"Blue");
            pros::screen::set_pen(pros::Color::red);
            if (selected_color == pros::Color::red){
                pros::screen::set_eraser(pros::Color::red);
                pros::screen::fill_rect(245,155,310,200);
            } else {
                pros::screen::set_eraser(pros::Color::black);
                pros::screen::draw_rect(245,155,310,200);
            }
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,262,170,"Red");
            last_selected_name = selected_auton.auton_data.Name;
            last_color = selected_color;
            if (status.touch_status == pros::E_TOUCH_RELEASED){
                for (const Auton_btn& auton : auton_buttons){
                    if (status.x >= auton.x1 && status.x <= auton.x2 && status.y >= auton.y1 && status.y <= auton.y2){
                        if (selected_auton.auton_data.Name == auton.auto_data.auton_data.Name){
                            if (status.press_count > presses){
                                mode = 4;
                                presses = status.press_count;
                            }       
                        }
                        presses = status.press_count;
                        selected_auton = auton.auto_data;
                        write_selector();
                    }
                }
                if (status.x >= 180 && status.x < 245 && status.y >= 165 && status.y <= 210){
                    selected_color = pros::Color::blue;
                    global_selected_color = 1;
                    write_color();
                } else if (status.x >= 245 && status.x < 310 && status.y >= 165 && status.y <= 210) {
                    selected_color = pros::Color::red;
                    global_selected_color = 2;
                    write_color();
                } else if (status.x >= 180 && status.x <= 320 && status.y >= 65 && status.y <= 140){
                    if (status.press_count > presses){
                        mode = 2;
                        presses = status.press_count;
                    }
                } else if (status.x >= 180 && status.x <= 300 && status.y >= 5 && status.y <= 50){
                    if (status.press_count > presses){
                        mode = 5;
                        presses = status.press_count;
                    }
            }                
            }
        } else if (mode ==2){
            pros::screen::set_eraser(pros::Color::black);
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_MEDIUM,10,10,"Heading: %.2f",chassis.odom_theta_get());
            pros::screen::print(pros::E_TEXT_MEDIUM,170,10,"X: %.2f",chassis.odom_x_get());
            pros::screen::print(pros::E_TEXT_MEDIUM,300,10,"Y: %.2f",chassis.odom_y_get());
            int front_distance_value = front_distance.get_distance();
            if (front_distance_value > 6000){
                front_distance_value = 0;
            }
            int front_distance_value2 = front_distance2.get_distance();
            if (front_distance_value2 > 6000){
                front_distance_value2 = 0;
            }
            int right_distance_value = right_distance.get_distance();
            if (right_distance_value > 6000){
                right_distance_value = 0;
            }
            int back_distance_value = back_distance.get_distance();
            if (back_distance_value > 6000){
                back_distance_value = 0;
            }
            int left_distance_value = left_distance.get_distance();
            if (left_distance_value > 6000){
                left_distance_value = 0;
            }
            pros::screen::print(pros::E_TEXT_MEDIUM,10,30,"Front Distance: %i mm  ,  %imm",front_distance_value,front_distance_value2);
            pros::screen::print(pros::E_TEXT_MEDIUM,10,50,"Right Distance: %i mm",right_distance_value);
            pros::screen::print(pros::E_TEXT_MEDIUM,10,70,"Back Distance: %i mm",back_distance_value);
            pros::screen::print(pros::E_TEXT_MEDIUM,10,90,"Left Distance: %i mm",left_distance_value);
            string tracker_value = "Vertical Odom Pod: " + util::to_string_with_precision(chassis.odom_tracker_left->get());             
            string tracker_width = "Offset: " + util::to_string_with_precision(chassis.odom_tracker_left->distance_to_center_get());
            pros::screen::print(pros::E_TEXT_MEDIUM,10,110,"%s",tracker_value);
            pros::screen::print(pros::E_TEXT_MEDIUM,10,130,"%s",tracker_width);
            // string tracker_value1 = "Horizontal Odom Pod: " + util::to_string_with_precision(chassis.odom_tracker_front->get());             
            // string tracker_width1 = " Offset: " + util::to_string_with_precision(chassis.odom_tracker_front->distance_to_center_get());
            // pros::screen::print(pros::E_TEXT_MEDIUM,10,130,"%s%s",tracker_value1,tracker_width1);
            pros::screen::print(pros::E_TEXT_MEDIUM,10,150,"Intake Voltage: %.2f",intake.get_power());
            pros::screen::print(pros::E_TEXT_MEDIUM,10,170,"Outtake Voltage: %.2f",outake.get_power());
            pros::screen::print(pros::E_TEXT_MEDIUM,10,190,"Color Sensor 1 Hue: %.0f",intake_color_1.get_hue());
            pros::screen::print(pros::E_TEXT_MEDIUM,10,210,"Color Sensor 2 Hue: %.0f",intake_color_2.get_hue());
            pros::screen::set_pen(pros::Color::dark_green);
            pros::screen::set_eraser(pros::Color::dark_green);
            pros::screen::fill_rect(340,80,460,160);
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_LARGE,350,105,"Reset");
            if (status.touch_status == pros::E_TOUCH_RELEASED){
                // if (status.x >= 270 && status.x <= 450 && status.y >= 160 && status.y <= 250){
                if (status.x >= 5 && status.x <= 200 && status.y >= 0 && status.y <= 40){
                    chassis.pid_tuner_toggle();
                    mode = 6;     
                    presses = status.press_count;
                } else if (status.x >= 340 && status.x <= 460 && status.y >= 80 && status.y <= 160){
                    chassis.pid_targets_reset();
                    chassis.drive_imu_reset();                  
                    chassis.drive_sensor_reset();
                    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
                    presses = status.press_count;
                } else {
                    if (status.press_count > presses){
                        mode = 1;
                        presses = status.press_count;
                    }
                }

                // }
            }

        } else if (mode ==3){
            pros::screen::set_eraser(pros::Color::black);
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_LARGE,55,20,"11005B: Winging It");
            if (color_name == true){
                pros::screen::set_pen(selected_color);
            }
            pros::screen::print(pros::E_TEXT_MEDIUM,10,160,"Samuel Bernath");
            pros::screen::print(pros::E_TEXT_MEDIUM,10,180,"Sebastian Mateo");
            pros::screen::print(pros::E_TEXT_MEDIUM,10,100,"Andrew Rothen");
            pros::screen::print(pros::E_TEXT_MEDIUM,10,120,"Paola Gouveia");
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_MEDIUM,10,80,"Aarav Bansal");
            pros::screen::print(pros::E_TEXT_MEDIUM,10,140,"Raghav Gaur");
            pros::screen::print(pros::E_TEXT_MEDIUM,10,200,"Valentina Botana");
            pros::screen::print(pros::E_TEXT_SMALL,318,80,"TERRA Evironmental");
            pros::screen::print(pros::E_TEXT_SMALL,315,100,"Research Institute");
            pros::screen::print(pros::E_TEXT_SMALL,355,120,"Miami, Forida");
            pros::screen::print(pros::E_TEXT_SMALL,260,140,"South Florida Amaze Award");
            pros::screen::print(pros::E_TEXT_SMALL,260,160,"147 Combined Skills Score");
            pros::screen::print(pros::E_TEXT_SMALL,340,180,"Global Rank 401");
            if (status.touch_status == pros::E_TOUCH_RELEASED){
                if (status.x >= 10 && status.x <= 100 && status.y >= 80 && status.y <= 160){
                    if (status.press_count > presses){
                        presses = status.press_count;
                        pros::screen::set_eraser(pros::Color::black);
                        pros::screen::erase();
                        color_name = true;
                    }
                }
                else {
                if (status.press_count > presses){
                    mode = 1;
                    color_name = false;
                    presses = status.press_count;
                }
            }
            }
        } else if (mode ==4){
            pros::screen::set_pen(selected_color);
            pros::screen::set_eraser(selected_color);
            pros::screen::fill_rect(50, 80, 170, 150);
            pros::screen::fill_circle(50, 115, 35);
            pros::screen::fill_circle(170, 115, 35);
            pros::screen::set_pen(pros::Color::white);
            pros::screen::print(pros::E_TEXT_LARGE,82,100,"Run");
            pros::screen::set_eraser(pros::Color::black);
            bool aligned = check_alignment();
            if (aligned == true){
                pros::screen::print(pros::E_TEXT_LARGE,250,50,"  Aligned");
            } else {
                pros::screen::print(pros::E_TEXT_LARGE,250,50,"Not Aligned");
            }
            if (status.touch_status == pros::E_TOUCH_RELEASED){
                if (status.x >= 15 && status.x <= 150 && status.y >= 80 && status.y <= 150){
                    autonomous();
                    mode =1;
                    presses = status.press_count;
                } else {
                    if (status.press_count > presses){
                        mode = 1;
                        presses = status.press_count;
                    }
                }
            }
            
        } else if (mode == 5){
            draw_frame(index);
            index++;
            if (index >frames){
                index = 0;
            }
            if (status.touch_status == pros::E_TOUCH_RELEASED){
                if (status.press_count > presses){
                //    mode  = 1;
                    if (mode != 5){
                        mode = 5;
                    } else if (mode == 5 && selected_animation < 4){
                        selected_animation++;
                    } else {
                        mode = 1;
                        selected_animation = 1;
                    }
                    presses = status.press_count;
                }
            }
            continue;
        } else {
            if (status.touch_status == pros::E_TOUCH_RELEASED){
                if (status.press_count > presses){
                    mode = 1;
                    presses = status.press_count;
                }
            }
            }
        pros::delay(delay_time);
    }
}