#include "autons.hpp"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <type_traits>
#include "EZ-Template/util.hpp"
#include "liblvgl/misc/lv_anim.h"
#include "main.h"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QLength.hpp"
#include "pros/colors.hpp"
#include "pros/device.hpp"
#include "pros/distance.hpp"
#include "pros/error.h"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "subsystems.hpp"
#include "screen.hpp"
#include "pid_graph.hpp"
#include <vector>
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 120;
const int TURN_SPEED = 90;
const int SWING_SPEED = 120;
///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(13.5, 0.0, 30);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(12.0, 0.0, 40.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.5, 0.0, 21.0, 0.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.8, 0.005, 70,10.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.7, 0.0, 72.25);  // Angular control for boomerang motions
  // chassis.pid_drive_constants_set(0.1, 0.0, 0.1);         // Fwd/rev constants, used for odom and non odom motions
  // chassis.pid_heading_constants_set(0.1, 0.0, 0.1);        // Holds the robot straight while going forward without odom
  // chassis.pid_turn_constants_set(0.1, 0.0, 0.1, 0.0);     // Turn in place constants
  // chassis.pid_swing_constants_set(0.1, 0.0, 0.1);           // Swing constants
  // chassis.pid_odom_angular_constants_set(0.1, 0.0, 0.1);    // Angular control for odom motions
  // chassis.pid_odom_boomerang_constants_set(0.1, 0.0, 0.1);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 90);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(6_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

okapi::QLength distance_to_wall(pros::Distance distance_sensor, okapi::QLength target){
  int raw_distance_mm = distance_sensor.get_distance();
  double angle_360 = chassis.odom_theta_get();
  double reference_angle_degrees = fmod(abs(angle_360),90.0);
  if (reference_angle_degrees >= 45){
    reference_angle_degrees = 90.0 - reference_angle_degrees;
  }
  double angle = util::to_rad(reference_angle_degrees);
  okapi::QLength move_amount;
  if (angle<0.035){
    float straight_distance = raw_distance_mm;
    okapi::QLength straight_distance_okapi = straight_distance * okapi::millimeter;
    move_amount = (straight_distance_okapi-target);
  } else {
    float straight_distance = (cos(abs(angle)) * raw_distance_mm);
    okapi::QLength straight_distance_okapi = straight_distance * okapi::millimeter;
    okapi::QLength drive_distance_straight = (straight_distance_okapi-target);
    okapi::QLength drive_distance = (drive_distance_straight/cos(abs(angle)));
    move_amount = drive_distance;
  }
  return move_amount;
}

okapi::QLength get_average_distance(int iterations, pros::Distance distance_sensor, int wait_time_total){
  int time_accum = 0;
  int wait_time = wait_time_total/iterations;
  double total_dist= 0.0001;
  while (time_accum < wait_time_total){
    double snapshot_dist = distance_sensor.get_distance();  
    if (snapshot_dist != PROS_ERR && snapshot_dist < 6000){
    total_dist += snapshot_dist;
    time_accum += wait_time;
    }
    pros::delay(wait_time);
  }
  okapi::QLength final_dist = (total_dist/iterations) * okapi::millimeter; 
  return final_dist;
}

okapi::QAngle get_front_angle_offset(){
  double x = get_average_distance(5, front_distance, 10).convert(okapi::inch);
  double y = get_average_distance(5, front_distance2, 10).convert(okapi::inch);
  double theta = atan((y-x)/front_sensor_offset);
  return theta * okapi::radian;
}

bool wrong_color(pros::Optical color_sensor_1,pros::Optical color_sensor_2){
  int hue1 = color_sensor_1.get_hue();
  int hue2 = color_sensor_2.get_hue();
  if (((150 <= hue1 && hue1 <= 220) or (150 <= hue2 && hue2 <= 220)) && global_selected_color!=1){ // If Color Detected is in the Blue Range and Blue is not the selected color
    return true;
  } else if (((15 <= hue1 && hue1 <= 30) or (15 <= hue2 && hue2 <= 30)) && global_selected_color!=2){ // If Color detected is in the Red Range and Red is not the selected co
    return true;
  } else {
    return false;
  }
}

void matchload(int delay_time){
  int time_accum = 0;
  intake.move(127);
  // chassis.pid_drive_set((get_average_distance(1, front_distance, 5)-230_mm),100);
  // chassis.pid_wait();
  while (time_accum < delay_time){
    if (wrong_color(intake_color_1,intake_color_2) == true){
      break;
    }
    else {
      pros::delay(5);
      time_accum += 5;
    }
  }
  intake.brake();
}

void matchload_skills(int delay_time,bool backup){
  intake.move(127);
  pros::delay(delay_time/3);
  okapi::QLength check_dist = get_average_distance(5, front_distance, 10);
  if (0.9*check_dist <= 210_mm && 210_mm <= 1.1*check_dist ){
    pros::delay(delay_time/2);
  } else {
    chassis.pid_drive_set(-5_in,100);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(7_in,100);
    chassis.pid_wait_quick();
    pros::delay(delay_time/3);
  }
  if (backup == true){
    chassis.pid_drive_set(-5_in,100);
    chassis.pid_wait_quick();
    chassis.pid_drive_set(7_in,100);
    chassis.pid_wait_quick();
    pros::delay(delay_time/3);
  } else {
    pros::delay(delay_time/3);
  }
  
}

void score_long(int delay_time){
  int time_accum = 0;
  outake.move(127);
  intake.move(127);
  pros::delay(250);
  intake.check = true;
  outake.check = true;
  pros::delay(delay_time-250);
  intake.brake();
  outake.brake();
  intake.check = false;
  outake.check = false;
}
okapi::QLength score_long_reset(int delay_time,int iterations, pros::Distance distance_sensor){
  int wait_interval = delay_time / iterations;
  int time_accum = 0;
  int backwards = 0;
  intake.move(127);
  outake.move(127);
  pros::delay(wait_interval*3);
  double total_distance = 0.00001;
  while (time_accum < delay_time){
    double snapshot_dist = distance_sensor.get_distance();
    if (snapshot_dist != PROS_ERR && snapshot_dist >= 4000){
      total_distance += snapshot_dist;
    }
    if (intake.get_power() <= 0.5 || outake.get_power() <= 0.5){
      backwards = 3;
    }
    if (backwards != 0){
      intake.move(-70);
      outake.move(-70);
      backwards -= 1;
      if (backwards ==0){
        intake.move(127);
        outake.move(127);
      }
    }
    pros::delay(wait_interval);
    time_accum += wait_interval;
  }
  return total_distance/iterations * okapi::millimeter;
}

void score_middle_skills(int delay_time,int out_speed){
  intake.move(127);
  outake.move(-out_speed);  
  int wait_interval = delay_time / 50;
  int time_accum = 0;
  pros::delay(wait_interval*10);
  time_accum+= wait_interval*10;
  while (time_accum < delay_time){
    if (intake.get_power() <= 0.5 || outake.get_power() <= 0.5){
      intake.move(-70);
      outake.move(-70);
      pros::delay(200);
      time_accum+= 200;
      outake.move(-out_speed);
      intake.move(127);
    }
    pros::delay(wait_interval);
    time_accum += wait_interval;

}
}

void score_middle(int delay_time,int out_speed){
  intake.move(127);
  outake.move(-out_speed);  
  int wait_interval = delay_time / 50;
  int time_accum = 0;
  pros::delay(wait_interval*5);
  time_accum+= wait_interval*5;
  while (time_accum < delay_time){
    if (intake.get_power() <= 0.5 || outake.get_power() <= 0.5){
      intake.move(-70);
      outake.move(-70);
      pros::delay(200);
      time_accum+= 200;
      outake.move(-out_speed);
      intake.move(127);
    }
    pros::delay(wait_interval);
    time_accum += wait_interval;

}
}

vector<double> distance_values_(string align_type){
  if (align_type == "null"){
    return {0,0,0,0};
  } else if (align_type == "left"){
    return {1225,420,0,0};
  } else if (align_type == "left_wall"){
    return {0,0,1000,5000};
  } else if (align_type == "right") {
    return {2000,500,0,0};
  } else if (align_type == "right_wall"){
    return {0,0,100,1260};
  } else {
    return {0,0,0,0};
  }
}

void matchload_test(){
  loader.set_value(true);
  pros::delay(500);
  intake.move(127);
  chassis.pid_drive_set((get_average_distance(1, front_distance, 5)-230_mm),80);
  chassis.pid_wait_quick_chain();
  loader.set_value(true);
  matchload(3000);
  chassis.pid_odom_set({{0_in,-10_in},rev,110});
  chassis.pid_wait();
  loader.set_value(false);
}

void test_auto(){
  while (true){
  if (wrong_color(intake_color_1, intake_color_2) == true){
    pros::screen::set_pen(pros::Color::red);
    pros::screen::fill_rect(0, 0, 500, 500);
    pros::delay(500);
   } else {  
    pros::screen::set_pen(pros::Color::white);
    pros::screen::fill_rect(0, 0, 500, 500);
    pros::delay(500);

  }
  pros::delay(10);
}
}

void angle_test(){
  okapi::QAngle offset = get_front_angle_offset();
  chassis.pid_turn_relative_set(offset,100);
  chassis.pid_wait();
}

void left_super_4(){  // Left Side Code; Aligns Against Wall; Picks up 3-Block Cluster; Scores 4 in Long Goal; Wing Push Into Control

  chassis.pid_odom_set({0.5_in,0_in,0_deg});
  intake.move(127); // Spins the intake
  chassis.pid_odom_set({ // Multistage Pure-Pursuit movement using Odom
    {{0_in,12_in},fwd,DRIVE_SPEED}, // Drives to the 3-Block cluster
    {{-8_in,24_in},fwd,DRIVE_SPEED}, // Drives through the blocks at an angle
    {{-33.5_in,6_in},fwd,DRIVE_SPEED}}, // Drives infront of the Long Goal
    false);
  chassis.pid_wait_until(11_in); // Deploys Loader to trap blocks once the robot reaches 30_in moved
  loader.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg,110); // Turns to absoulte heading 180 to align with Long Goal
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-35_in,15_in},rev,110},false); // Drives into Long Goal
  chassis.pid_wait_quick(); 
  score_long(1500);
  outake.brake();
  intake.brake();
  chassis.pid_odom_set({{-35_in,6_in},fwd,110},false); // Drives back and turns
  chassis.pid_wait_quick();
  chassis.pid_turn_relative_set(-50_deg,TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({
    {{-43_in,9.3_in},rev,120},
    {{-43_in,35_in},rev,110}},
    false);
  chassis.pid_wait();
}

void left_7(){ // Left Side 7-Block Goal Rush; Aligns With Wall; Picks up 3-Block Cluster; Picks up 3 from Matchloader; Scores 7 in Long Goal and holds with Hood
  chassis.pid_odom_set({0.5_in,0_in,0_deg});
  intake.move(127); // Spins the intake
  chassis.pid_odom_set({ // Multistage Pure-Pursuit movement using Odom
    {{0_in,12_in},fwd,DRIVE_SPEED}, // Drives to the 3-Block cluster
    {{-8_in,24_in},fwd,DRIVE_SPEED}, // Drives through the blocks at an angle
    {{-33_in,0_in},fwd,DRIVE_SPEED}}, // Drives infront of the Long Goal
    false);
  chassis.pid_wait_until(11_in); // Deploys Loader to trap blocks once the robot reaches 30_in moved
  loader.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg,110); // Turns to absoulte heading 180 to align with Match Loader
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{-35.5_in,-16_in},fwd,75},false); // Slowly drives into matchloader to not shoot out blocks
  chassis.pid_wait_quick();
  matchload(400);
  intake.brake();
  chassis.pid_odom_set({{-35_in,13.5_in},rev,110},false); // Drives into Long Goal
  chassis.pid_wait_quick();
  score_long(10000);
}

void left_split(){ // Left Side Split; Aligns on Park Brace; Picks up 3 from Matchloader; Scores 4 in Long Goal; Picks up 3-Block Cluster; Scores 3 in Upper Middle Goal; Wing Push on Long Goal

  intake.move(127); // Spins the intake
  chassis.pid_odom_set({{0_in,24_in},fwd,120},true); // Drives to between Matchloader and Long Goal
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90_deg,100); // Turns to align with Matchloader
  loader.set_value(true); // Puts down loader 
  chassis.pid_wait_quick();
  // pros::delay(2000);
  chassis.pid_odom_set({{-12_in,29.8_in},fwd,80},false); // Slowly drives into Matchloader to not shoot out blocks
  chassis.pid_wait_quick();
  matchload(600);
  intake.brake();
  loader.set_value(false);
  chassis.pid_odom_set({{13_in,30_in},rev,110},true); // Drives into Long Goal
  chassis.pid_wait_until({-10_in,30_in});
  loader.set_value(false);
  // intake.move(-127);
  // pros::delay(50);
  chassis.pid_wait_until({0_in,30_in});
  intake.move(127);
  chassis.pid_wait_quick_chain();
  score_long(1750);
  intake.brake();
  outake.brake();
  chassis.pid_odom_set({{2.25_in,30_in},fwd,120});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(135_deg,100);
  chassis.pid_wait_quick();
  intake.move(127);
  outake.move(-127);
  chassis.pid_drive_set(30_in,120);
  chassis.pid_wait_until(20_in);
  loader.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-45_deg,100);
  chassis.pid_wait_quick();
  outake.brake();
  chassis.pid_drive_set(-18_in,120);
  chassis.pid_wait_quick_chain();
  middle_hatch.set_value(true);
  score_middle(1400, 110);
  loader.set_value(false);
  intake.brake();
  outake.brake();
  chassis.pid_drive_set(33.5_in,110);
  chassis.pid_wait_quick();
  middle_hatch.set_value(false);
  chassis.pid_turn_set(90_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(32_in,120);
  chassis.pid_wait();
  // chassis.pid_odom_set({ // Odom Pure-Pursuit
  //   {{2.5_in,31_in},fwd,120}, //Drives out of Long Goal
  //   {{22_in,8_in},fwd,120}}, // Drives into 3-Block Cluster
  //   false);
  // intake.move(127); // Spins Intake
  // chassis.pid_wait_until(18_in);
  // loader.set_value(true); // Deploy loader to trap blocks
  // chassis.pid_wait_quick();
  // chassis.pid_turn_relative_set(175_deg,TURN_SPEED); // Turns to align Hatch with Upper Middle Goal
  // chassis.pid_wait_quick();
  // chassis.pid_odom_set({{36.25_in,-5.8_in},rev,110});  // Drive into Upper Middle Goal
  // chassis.pid_wait_quick();
  // middle_hatch.set_value(true); // Lower Hatch to score in Upper Middle Goal
  // intake.move(-127);
  // outake.move(-127);
  // pros::delay(100);
  // intake.move(127);
  // outake.move(-100);
  // // score_middle(1750, 100);
  // pros::delay(1750); // Wait for all blocks to be scored
  // intake.brake();
  // outake.brake();
  // loader.set_value(false); // Puts up Sweeper, closes Hatch, puts up Wing
  // middle_hatch.set_value(false);
  // chassis.pid_odom_set({{15_in,15_in},fwd,120},false); // Drives back to next to Long Goal
  // chassis.pid_wait_quick();
  // chassis.pid_turn_set(85_deg,TURN_SPEED); // Turns to align with Long Goal
  // chassis.pid_wait_quick(); 
  // chassis.pid_odom_set({{38_in,15_in},fwd,127},false); // Drives forward to push with Wing
  // chassis.pid_wait_quick();
}

void right_super_4(){ // Right Side Code; Aligns Against Wall; Picks up 3-Block Cluster; Scores 4 in Long Goal; Wing Push Into Control
  chassis.pid_odom_set({-0.5_in,0_in,0_deg});
  intake.move(127); // Spins the intake throughout the entire code
  chassis.pid_odom_set({ // Multistage Pure-Pursuit movement using Odom
    {{0_in,12_in},fwd,DRIVE_SPEED}, // Drives to the 3-Block cluster
    {{8_in,24_in},fwd,DRIVE_SPEED}, // Drives through the blocks at an angle
    {{24_in,6_in},fwd,DRIVE_SPEED}} // Drives infront of the Long Goal
    ,false);
  chassis.pid_wait_until(15_in);
  loader.set_value(true); // Deploys Loader to trap blocks once the robot reaches 30_in moved
  // wing.set_value(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg,110); // Turns to absoulte heading 180 to align with Long Goal
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{28_in,13_in},rev,110},false); // Drives into Long Goal
  chassis.pid_wait_quick(); 
  score_long(1500);  // Waits for blocks to be scored
  intake.brake();
  outake.brake();
  chassis.pid_odom_set({{28_in,6_in},fwd,110},false); // Drives back and turns
  chassis.pid_wait_quick();
  chassis.pid_turn_relative_set(-50_deg,TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({
    {{21_in,9.3_in},rev,120},
    {{21_in,30_in},rev,110}},
    false);
  chassis.pid_wait();
  // wing.set_value(false); // Put Wing down
  // chassis.pid_drive_set(10_in,120);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_turn_set(180_deg,TURN_SPEED);
  // chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(20_in,127); // Realigns with Long Goal and pushes blocks with the Wing into control
  // chassis.pid_wait();

}

void right_7(){ // Right Side 7-Block Goal Rush; Aligns With Wall; Picks up 3-Block Cluster; Picks up 3 from Matchloader; Scores 7 in Long Goal and holds with Hood
  chassis.pid_odom_set({-0.5_in,0_in,0_deg});
  intake.move(127); // Spins the intake
  chassis.pid_odom_set({ // Multistage Pure-Pursuit movement using Odom
    {{0_in,12_in},fwd,120}, // Drives to the 3-Block cluster
    {{8_in,24_in},fwd,120}, // Drives through the blocks at an angle
    {{26_in,-6_in},fwd,120}}, // Drives infront of the Long Goal
    true);
  chassis.pid_wait_until(15_in); // Deploys Loader to trap blocks once the robot reaches 30_in moved
  loader.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg,110); // Turns to absoulte heading 180 to align with Match Loader
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{28_in,-16_in},fwd,60},false); // Slowly drives into matchloader to not shoot out blocks
  chassis.pid_wait_quick();
  matchload(600);
  // pros::delay(100); // Waits to pickup blocks
  // intake.brake();
  chassis.pid_odom_set({{29_in,13.5_in},rev,120},false); // Drives into Long Goal
  chassis.pid_wait_quick_chain();
  intake.move(127); // Scores 7
  outake.move(127);
  score_long(5000);
  chassis.pid_drive_set(2_in,127);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3_in,127);
  chassis.pid_wait_quick_chain();
}

void solo_win_point(){
  intake.move(127); // Spins the intake
  chassis.pid_odom_set({{0_in,20_in},fwd,120},true); // Drives to between Matchloader and Long Goal
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg,110); // Turns to align with Matchloader
  loader.set_value(true); // Puts down loader 
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{10.5_in,26.5_in},fwd,70},false); // Slowly drives into Matchloader to not shoot out blocks
  chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(get_average_distance(1, front_distance, 2)-230_mm,70);
  // chassis.pid_wait_quick();
  matchload(400); // Matchloads with color sort
  chassis.pid_odom_set({{-19_in,27_in},rev,120},true); // Drives into Long Goal
  chassis.pid_wait_until(-10_in);
  loader.set_value(false);
  chassis.pid_wait_quick();
  loader.set_value(false);
  score_long(1500);
  okapi::QLength new_x = get_average_distance(20, left_distance, 100)-445_mm;
  chassis.odom_pose_set({new_x,0_in,0_deg});
  intake.brake();
  outake.brake(); 
  chassis.pid_odom_set({{0_in,7_in},fwd,120});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-90_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-101_in,120);
  chassis.pid_wait_quick();
  loader.set_value(true);
  chassis.pid_turn_set(0_deg,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{92_in,25_in},fwd,120});
  chassis.pid_wait_quick_chain();
  matchload(750);
  chassis.pid_odom_set({{92.5_in,0_in},rev,120});
  chassis.pid_wait_quick_chain();
  score_long(1500);
  okapi::QLength new_x2 = get_average_distance(20, right_distance, 100)-445_mm;
  chassis.odom_pose_set({new_x2,0_in,0_deg});
  outake.brake();
  chassis.pid_drive_set(10_in,120);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(-135_deg,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(30_in,110);
  chassis.pid_wait_until(20_in);
  loader.set_value(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(45_deg,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-10_in,120);
  chassis.pid_wait_quick_chain();
  middle_hatch.set_value(true);
  outake.move(-127);

  

}

void solo_win_point3(){
  chassis.pid_odom_set({0.5_in,0_in,0_deg});
  intake.move(127); // Spins the intake
  outake.move(-127);
  chassis.pid_odom_set({
    {{0_in,14_in},fwd,127},
    {{-5.25_in,22.25_in},fwd,127}},true);
  chassis.pid_wait_until(11_in); // Deploys Loader to trap blocks once the robot reaches 30_in moved
  loader.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-135_deg,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-14.5_in,100);
  chassis.pid_wait_until(-13.5_in);
  middle_hatch.set_value(true);
  // pros::delay(250);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  outake.move(-100);
  score_middle(1400, 90);
  // pros::delay(1500);
  intake.brake();
  outake.brake();
  chassis.pid_drive_set(12_in,100);
  middle_hatch.set_value(false);
  loader.set_value(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(85_deg,100);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  outake.move(-100);
  chassis.pid_odom_set({{32_in,21_in},fwd,127});
  chassis.pid_wait_until({17_in,21_in});
  loader.set_value(true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-40_deg,100);
  chassis.pid_wait_quick_chain();
  loader.set_value(false);
  chassis.pid_drive_set(12.5_in,100);
  chassis.pid_wait_quick_chain();
  pros::delay(250);
  low_goal_arm.set_value(true);
  intake.move(-100);
  outake.move(-127);
  pros::delay(1500);
  outake.brake();
  intake.brake();
  chassis.pid_drive_set(-41_in,100);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(188_deg,100);
  loader.set_value(true);
  low_goal_arm.set_value(false);
  chassis.pid_wait_quick();
  // okapi::QLength new_x = get_average_distance(5, left_distance, 10) - 460_mm;
  // chassis.odom_pose_set({new_x,chassis.odom_y_get()*okapi::inch,chassis.odom_theta_get()*okapi::degree});
  // master.print(0, 0, "%d", new_x.convert(okapi::inch));
  // chassis.pid_odom_set({{0_in,-7.5_in},fwd,100});
  // chassis.pid_wait_quick_chain();
  // matchload(800);
  // intake.move(127);
  // chassis.pid_odom_set({{2_in,16_in},rev,127});
  // chassis.pid_wait_quick_chain();
  // score_long(10000);
  chassis.pid_odom_set({{58_in,-7.5_in},fwd,100});
  chassis.pid_wait_quick_chain();
  matchload(800);
  intake.move(127);
  chassis.pid_odom_set({{60_in,16_in},rev,127});
  chassis.pid_wait_quick_chain();
  score_long(10000);


}

void solo_win_point2(){
  intake.move(127); // Spins the intake
  chassis.pid_odom_set({{0_in,20_in},fwd,120},true); // Drives to between Matchloader and Long Goal
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(90_deg,110); // Turns to align with Matchloader
  loader.set_value(true); // Puts down loader 
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{10.5_in,26.5_in},fwd,70},false); // Slowly drives into Matchloader to not shoot out blocks
  chassis.pid_wait_quick_chain();
  // chassis.pid_drive_set(get_average_distance(1, front_distance, 2)-230_mm,70);
  // chassis.pid_wait_quick();
  matchload(400); // Matchloads with color sort
  chassis.pid_odom_set({{-19_in,27_in},rev,120},true); // Drives into Long Goal
  chassis.pid_wait_until(-10_in);
  loader.set_value(false);
  chassis.pid_wait_quick();
  loader.set_value(false);
  score_long(1500);
  okapi::QLength new_x = get_average_distance(20, left_distance, 100)-445_mm;
  chassis.odom_pose_set({new_x,0_in,0_deg});
  intake.brake();
  outake.brake();
  chassis.pid_odom_set({{0_in,8_in},fwd,120});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(133_deg,100);
  chassis.pid_wait_quick();
  intake.move(127);
  low_goal_arm.set_value(true);
  // chassis.pid_odom_set({
  //   {{17_in,-0.75_in},fwd,127},
  //   {{19_in,-8_in},fwd,127},
  //   {{32_in,-19.5_in},fwd,127}},true);
  // chassis.pid_wait_until_point({13_in,3_in});
  chassis.pid_drive_set(56_in,100);
  // chassis.pid_wait_until({12_in,1_in});
  chassis.pid_wait_until(12_in);
  loader.set_value(true);
  chassis.pid_wait_until(35_in);
  chassis.pid_speed_max_set(70);
  loader.set_value(false);
  // intake.brake();
  chassis.pid_wait_quick();
  intake.move(-127);
  pros::delay(2000);
  chassis.pid_speed_max_set(127);
  intake.brake();
  

}

void skills_old(){
  intake.move(127); // Spins the intake
  chassis.pid_odom_set({{0_in,29.25_in},fwd,120},true); // Drives to between Matchloader and Long Goal
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-85_deg,110); // Turns to align with Matchloader
  chassis.pid_wait_quick_chain();
  loader.set_value(true); // Puts down loader 
  chassis.pid_odom_set({{-11_in,29.25_in},fwd,50},false); // Slowly drives into Matchloader to not shoot out blocks
  chassis.pid_wait();
  intake.move(127);
  pros::delay(1500);
  chassis.pid_drive_set(-5_in,120);
  chassis.pid_wait_quick_chain();
  intake.move(-50);
  pros::delay(200);
  intake.move(127);
  chassis.pid_drive_set(6_in,60);
  chassis.pid_wait_quick_chain();
  intake.move(127);
  pros::delay(750);
  chassis.pid_odom_set({{0,29.25},rev,120},false);
  chassis.pid_wait();
  chassis.pid_turn_set(-50_deg,100);
  chassis.pid_wait_quick();
  loader.set_value(false);
  intake.brake();
  chassis.pid_odom_set({{14_in,16_in},rev,127},  true);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-87.5_deg,100);
  chassis.pid_wait();
  chassis.pid_drive_set(-82_in,110,true);
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg,100);
  chassis.pid_wait();
  // chassis.pid_odom_set({{87_in,30.5_in},rev,100});
  // chassis.pid_wait();
  chassis.pid_drive_set(-(get_average_distance(100, back_distance, 500)-15_in),80);
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg,100);
  chassis.pid_wait();
  // okapi::QLength from_wall = get_average_distance(
  //   100, back_distance, 400) - 16.75_in;
  chassis.pid_drive_set(-13_in,100);
  // chassis.odom_pose_set({0_in,from_wall,0_deg});
  // master.print(0, 0, "%s",std::to_string(from_wall.convert(okapi::inch)));
  // chassis.pid_odom_set({{0_in,-10_in},rev,100},false);
  chassis.pid_wait_quick();
  // chassis.pid_turn_set(90_deg,100);
  // chassis.pid_wait_quick();
  // chassis.pid_odom_set({{71,from_wall}})
  intake.move(127);
  outake.move(127);
  pros::delay(500);
  if (intake.get_power() <= 0.5 or outake.get_power() <= 0.5){
    intake.move(-70);
    outake.move(-70);
    pros::delay(100);
    outake.move(127);
    intake.move(127);
  }
  okapi::QLength new_x = get_average_distance(100,left_distance,2000) - 16.9_in;
  intake.brake();
  outake.brake();
  // master.print(0, 0, "%s",std::to_string(new_x.convert(okapi::inch)));
  chassis.odom_pose_set({new_x,0_in,0_deg});
  loader.set_value(true);
  intake.move(127);
  chassis.pid_odom_set({
    {{0_in,15_in},fwd,120},
    {{0.75_in,28.5_in},fwd,60}
    },false);
  chassis.pid_wait_quick();
  chassis.pid_wait_quick();
  pros::delay(1500);
  intake.brake();
  chassis.pid_odom_set({{0_in,-1_in},rev,100});
  chassis.pid_wait_quick();
  intake.move(127);
  outake.move(127);
  pros::delay(500);
  if (intake.get_power() <= 0.5 || outake.get_power() <= 0.5){
    intake.move(-70);
    outake.move(-70);
    pros::delay(150);
    outake.move(127);
    intake.move(127);
  }
  okapi::QLength new_x_1 = get_average_distance(100,left_distance,2000) - 16.9_in;
  intake.brake();
  outake.brake();
  // master.print(0, 0, "%s",std::to_string(new_x_1.convert(okapi::inch)));
  chassis.odom_pose_set({new_x_1,0_in,0_deg});
  chassis.pid_odom_set({{0_in,10_in},fwd,120});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-107_in,110);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-(get_average_distance(20, back_distance, 300)-15.5_in),100);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(0_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-13_in,100);
  // okapi::QLength from_wall_matchloader = get_average_distance(50,right_distance,300) - 17.6_in;
  // master.print(0, 0, "%s", std::to_string(from_wall_matchloader.convert(okapi::inch)));
  // chassis.odom_x_set(from_wall_matchloader);
  // chassis.pid_odom_set({{(-1_in),-4_in},rev,100});
  chassis.pid_wait_quick();
  okapi::QLength new_x_5 = get_average_distance(30, right_distance, 300)-17.6_in;
  chassis.odom_pose_set({new_x_5,0_in,0_deg});
  intake.move(127);
  chassis.pid_odom_set({
    {{0_in,15_in},fwd,120},
    {{-1_in,28.5_in},fwd,60}
    },false);
  chassis.pid_wait_quick();
  pros::delay(2000);
  // chassis.pid_drive_set(-5_in,120);
  // chassis.pid_wait_quick_chain();
  // intake.move(-50);
  // pros::delay(200);
  // intake.move(127);
  // chassis.pid_drive_set(6_in,60);
  // chassis.pid_wait_quick_chain();
  // intake.move(127);
  // pros::delay(750);
  intake.brake();
  chassis.pid_odom_set({{0_in,8_in},rev,110});
  chassis.pid_wait_quick();
  loader.set_value(false);
  chassis.pid_turn_set(45_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{-15_in,0_in},rev,120});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(0_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-75_in,120);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-(get_average_distance(30, back_distance, 300)-15_in),100);
  chassis.pid_wait_quick();
  // chassis.pid_odom_set({{0_in,-70_in},rev,120});
  // chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg,120);
  chassis.pid_wait();
  // okapi::QLength from_wall_2 = get_average_distance(50,left_distance,400) - 16.9_in;
  // chassis.pid_odom_set({{from_wall_2,-48_in},rev,90});
  // chassis.pid_wait_quick();
  chassis.pid_drive_set(-10_in,100);
  chassis.pid_wait_quick();
  intake.move(127);
  outake.move(127);
  pros::delay(500);
  if (intake.get_power() <= 0.5 || outake.get_power() <= 0.5){
    intake.move(-70);
    outake.move(-70);
    pros::delay(150);
    outake.move(127);
    intake.move(127);
  }
  okapi::QLength new_x_3 = get_average_distance(100,left_distance,2000) - 16.9_in;
  intake.brake();
  outake.brake();
  master.print(0, 0, "%s",std::to_string(new_x_3.convert(okapi::inch)));
  chassis.odom_pose_set({new_x_3,0_in,0_deg});
  loader.set_value(true);
  intake.move(127);
  chassis.pid_odom_set({
    {{0_in,15_in},fwd,120},
    {{1_in,28_in},fwd,60}
    },false);
  chassis.pid_wait_quick();
  chassis.pid_wait_quick();
  pros::delay(2000);
  // chassis.pid_drive_set(-5_in,120);
  // chassis.pid_wait_quick_chain();
  // intake.move(-50);
  // pros::delay(200);
  // intake.move(127);
  // chassis.pid_drive_set(6_in,60);
  // chassis.pid_wait_quick_chain();
  // intake.move(127);
  // pros::delay(750);
  intake.brake();
  chassis.pid_odom_set({{0_in,0_in},rev,100});
  chassis.pid_wait_quick();
  intake.move(127);
  outake.move(127);
  pros::delay(500);
  if (intake.get_power() <= 0.5 || outake.get_power() <= 0.5){
    intake.move(-70);
    outake.move(-70);
    pros::delay(150);
    outake.move(127);
    intake.move(127);
  }
  okapi::QLength new_x_4 = get_average_distance(100,left_distance,2000) - 16.9_in;
  intake.brake();
  outake.brake();
  master.print(0, 0, "%s",std::to_string(new_x_4.convert(okapi::inch)));
  chassis.odom_pose_set({new_x_4,0_in,0_deg});
  loader.set_value(false);
  // chassis.pid_odom_set({
  //   {{0_in,5_in},fwd,120},
  //   {{45_in,10_in},fwd,120}},false);
  chassis.pid_drive_set(10_in,100);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(90_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(45_in,100);  
 chassis.pid_wait_quick();
  chassis.pid_turn_set(0_deg,100);
  chassis.pid_wait_quick();
  intake.move(127);
  outake.move(127);
  chassis.pid_drive_set(-5,127);
  chassis.pid_wait();
  chassis.pid_drive_set(100_in,200);
  chassis.pid_wait();


  
}

void skills(){
  intake.move(127);
  pros::delay(100);
  chassis.pid_drive_set(2_in,100);
  chassis.pid_wait();
  pros::delay(100);
  chassis.pid_drive_set(10_in,90);
  chassis.pid_wait();
  chassis.pid_drive_set(-5_in,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(8_in,100);
  chassis.pid_wait_quick();
  pros::delay(250);
  chassis.pid_drive_set(-22_in,120);
  chassis.pid_wait();
  intake.brake();
  okapi::QLength new_y = get_average_distance(50, front_distance, 250)-1020_mm;
  chassis.pid_drive_set(new_y-4_in,110);
  chassis.pid_wait_quick();
  chassis.odom_pose_set({0_in,0_in,0_deg});
  chassis.pid_turn_set(90_deg,100);
  chassis.pid_wait_quick();
  intake.move(127);
  chassis.pid_drive_set(22_in,80);
  chassis.pid_wait_quick();
  // pros::delay(100);
  // intake.brake();
  chassis.pid_drive_set(2_in,120);
  // chassis.pid_turn_set(-45_deg,100);
  chassis.pid_wait_quick();
  // chassis.pid_drive_set(-18_in,120);
  // chassis.pid_wait_quick();
  chassis.pid_turn_set(45_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-13_in,120);
  chassis.pid_wait_quick();
  outake.move(127);
  pros::delay(300);
  outake.brake();
  loader.set_value(true);
  intake.move(-70);
  outake.move(-70);
  pros::delay(300);
  middle_hatch.set_value(true);
  intake.move(127);
  outake.move(-80);
  score_middle_skills(3500, 80);
  intake.move(-127);
  outake.brake();
  chassis.pid_drive_set(46_in,120);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(0_deg,100);
  chassis.pid_wait_quick();
  loader.set_value(true);
  middle_hatch.set_value(false);
  intake.move(127);
  chassis.pid_drive_set(get_average_distance(5, front_distance, 20)-210_mm+1.75_in,60);
  chassis.pid_wait_quick_chain();
  matchload_skills(2500, true);
  intake.brake();
  chassis.odom_pose_set({0_in,0_in,chassis.odom_theta_get()*okapi::degree});
  chassis.pid_odom_set({{0_in,-10_in},rev,120});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-45_deg,100);
  chassis.pid_wait_quick();
  wing.set_value(true);
  chassis.pid_odom_set({{12_in,-24_in},rev,120});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(0_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-68_in,120);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-90_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-(get_average_distance(20, back_distance, 50)-480_mm),110);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(-180_deg,100);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-10_in,100);
  chassis.pid_wait_quick_chain();
  score_long(3000);
  okapi::QLength new_x = get_average_distance(20,left_distance,100) - 444_mm;
  chassis.odom_pose_set({new_x,0_in,0_deg});

}






///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches
  // pros::Task graph_task([&]() {start_graph(0, 0, 0, chassis.fwd_rev_drivePID);});
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void odom_drive_test(){
  // pros::Task graph_task([&]() {start_graph(0, 10, 0, chassis.fwd_rev_drivePID);});
  chassis.pid_odom_set({{0_in,10_in},fwd,100});
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg,100);
  chassis.pid_wait();
  chassis.pid_odom_set({{-10_in,10_in},fwd,100});
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_odom_set({{0_in,0_in,0_deg},rev,100});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg,100);
  chassis.pid_wait();
}

void turn_test(){
  // pros::Task graph_task([&]() {start_graph(0, 0, 90, chassis.turnPID);});
  chassis.pid_turn_set(90_deg,100);
  chassis.pid_wait();
}

void odom_long_drive(){
  // pros::Task graph_task([&]() {start_graph(0, 50, 0, chassis.fwd_rev_drivePID);});
  chassis.pid_odom_set({{0_in,50_in},fwd,120});
  chassis.pid_wait();
  chassis.pid_odom_set({{0_in,0_in},rev,120});
  chassis.pid_wait();
}


///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  pros::Task graph_task([&]() {start_graph(0, 0, 0, chassis.fwd_rev_drivePID);});
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       false);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       false);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

