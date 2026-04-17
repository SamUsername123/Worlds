#include "main.h"
#include <cstddef>
#include <cstdio>
#include <string>
#include "EZ-Template/auton.hpp"
#include "EZ-Template/util.hpp"
#include "autons.hpp"
#include "okapi/api/units/QLength.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"
#include "screen.hpp"
#include "subsystems.hpp"
#include "controller.hpp"
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-11, -12, -13},     // Left Chassis Ports (negative port will reverse it!)
    {18, 19, 20},  // Right Chassis Ports (negative port will reverse it!)

    2,      // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    600);   // Wheel RPM = cartridge * (motor gear / wheel gear)
vector<pros::Motor> drivetrain_motors = {pros::Motor(11),pros::Motor(12),pros::Motor(13),pros::Motor(18),pros::Motor(19),pros::Motor(20)};
// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2, 4.0);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(6, 2, -1.04);   // This tracking wheel is parallel to the drive wheels

pros::Motor boring_intake(1);
pros::Motor boring_outake(-10);


antijam_motor intake = antijam_motor(&boring_intake,50,100,55.0);
antijam_motor outake = antijam_motor(&boring_outake,50,100,60.0);

// pros::Motor intake(1);
// pros::Motor outake(-10);

pros::adi::DigitalOut loader(1);
pros::adi::DigitalOut wing(3);
pros::adi::DigitalOut middle_hatch(2);
pros::adi::DigitalOut low_goal_arm(4);

pros::Distance front_distance(5);
pros::Distance front_distance2(15);
pros::Distance back_distance(8);
pros::Distance right_distance(7);
pros::Distance left_distance(3);

pros::Optical intake_color_1(4);
pros::Optical intake_color_2(14);

bool wingValue = false;
bool loaderValue = false;
bool middleValue = false;
bool lowValue = false;

vector<pros::Distance> dist_list = {left_distance,back_distance,front_distance,right_distance};
int dist_selected = 0;

// Better_Auton::Better_Auton() : auton_data("", nullptr), distances({}) {}

vector<Better_Auton> autons = {
  // {ez::Auton{"Pursuit",odom_pure_pursuit_example},distance_values_("null")},
  // {ez::Auton{"Boom",odom_boomerang_example},distance_values_("null")},
  // {"Left",left_split},
  // {ez::Auton{"Angle Test",angle_test},distance_values_("null")},
  // {ez::Auton{"Left 7",left_7},distance_values_("left")},
  // {ez::Auton{"Odom Long",odom_long_drive},distance_values_("null")},
  // {ez::Auton{"Matchload",matchload_test},distance_values_("null")},
  // {ez::Auton{"Offsets",measure_offsets},distance_values_("null")},
  // {"Left 7",left_7},
  {ez::Auton{"Left Split",left_split},distance_values_("left")},
  {ez::Auton{"Left 4",left_super_4},distance_values_("left_wall")},
  {ez::Auton{"Left 7",left_7},distance_values_("left")},
  {ez::Auton{"Solo",solo_win_point3},distance_values_("rght_wall")},
  // {ez::Auton{"Solo",solo_win_point},distance_values_("right_wall")},
  {ez::Auton{"Right 7",right_7},distance_values_("right_wall")},
  {ez::Auton{"Right 4",right_super_4},distance_values_("right_wall")},
  {ez::Auton{"Skills",skills},distance_values_("null")},
};
Better_Auton selected_auton = autons[0]; 
int global_selected_color = 1;
int mode = 1;
int selected_animation = 1;

void start_controller_task(){
    pros::delay(1000);
    main_controller_task();
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();
  boring_intake.brake();
  pros::delay(500);  // Stop the user from doing anything while legacy ports configure
  chassis.pid_tuner_disable();
  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_front_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  chassis.odom_tracker_left_set(&vert_tracker);
  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.2);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();
  FILE * f = fopen("/usd/auton.txt", "r");
  char buffer[50];
  string last_name;
  if (f){
    if (fgets(buffer, sizeof(buffer), f)!= NULL){
      fclose(f);
    }
    std::string str(buffer);
    size_t last = str.find_last_not_of("\r\n");
    last_name = (last == std::string::npos) ? "" : str.substr(0,last+1);
    printf("%s\n",last_name.c_str());
    for (const Better_Auton& auton : autons){
      if (auton.auton_data.Name == last_name){
      selected_auton = auton;
    }
  }
  }
  
  FILE * f2 = fopen("/usd/color.txt", "r");
  char buffer2[50];
  string last_color;
  if (f2){
    if (fgets(buffer2, sizeof(buffer2), f2)!= NULL){
      fclose(f2);
    }
    std::string str1(buffer2);
    size_t last1 = str1.find_last_not_of("\r\n");
    last_color = (last1 == std::string::npos) ? "" : str1.substr(0,last1+1);
    printf("%s\n",last_color.c_str());
    if (last_color == "blue"){
      global_selected_color = 1;
    } else if (last_color == "red"){
      global_selected_color = 2;
    } else {
      global_selected_color = 1;
    }
  }
  pros::Task ezScreenTask(ez_screen_task);
  pros::Task jamtask([&]() {start_antijam_task();});

  intake_color_1.set_led_pwm(100);
  intake_color_2.set_led_pwm(100);
  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU




  // Initialize chassis and auton selector
  chassis.initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  // pros::Task controller_task(start_controller_task);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  low_goal_arm.set_value(false);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */
   // Calls selected auton from autonomous selector
  selected_auton.auton_data.auton_call();
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  start_selector(autons,selected_auton);
  // while (true) {
  //   pros::screen::set_pen(pros::c::COLOR_BLACK);
  //   pros::c::screen_fill_rect(0, 0, 500, 500);
  //   pros::screen::set_pen(pros::Color::white);
  //   pros::screen::print(pros::E_TEXT_MEDIUM,50,50,"Heading %s",std::to_string(chassis.odom_theta_get()));
  //   pros::delay(ez::util::DELAY_TIME);
  // }
}


/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
    //   // master.print(0, 0, "Front: %s", std::to_string(get_average_distance(10, front_distance, 200)));
    //   // pros::delay(200);
    //   int dist_index = fmod(dist_selected,4);

    //   double average_dist = get_average_distance(1000,dist_list[dist_index],10000).convert(okapi::inch);
    //   switch (dist_index){
    //     case 0:
    //     master.print(0,0,"Left %s",std::to_string(average_dist));
    //     case 1:
    //     master.print(0,0,"Back %s",std::to_string(average_dist));
    //     case 2:
    //     master.print(0,0,"Front %s",std::to_string(average_dist));
    //     case 3:
    //     master.print(0,0,"Right %s",std::to_string(average_dist));
    //   }
    // dist_selected += 1;
    // pros::delay(100);
    // }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) && master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      loader.set_value(false);
      middle_hatch.set_value(false);
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }


  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



void opcontrol() {
  chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST);
  int time_accum;
  int hold_count;
  string pattern;
  int vibration_time_accum = 0 ;
  intake.check = true;
  outake.check = true;
  // low_goal_arm.set_value(true);
  while (true) {
    // printf(">Error:%.2f\n",chassis.fwd_rev_drivePID.error);
    double motor_power = intake.get_power(); // Reads how much wattage the motor is pulling, used for jam detection

    ez_template_extras();

    chassis.opcontrol_tank();  // Tank control

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) // Checks if a button being pressed requires intake forward movement
    {
      intake.move(127);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) // Spin out the intake out
    {
      if (lowValue == true)
      {
        intake.move(-127);
      } 
      else
      {
      intake.move(-127);
      }
    }
    else
    {
      hold_count = 0;
      time_accum = 0;
      intake.brake();
    }
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) // Spin the outake
    {
      if (middleValue == true)
        {
          outake.move(-127);
        } 
      else
        {
        outake.move(127);
        }
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      outake.move(-127);
    }
    else
    {
      outake.brake();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) // Toggle Wing position
    {
      if(wingValue == false)
      {
          wing.set_value(true);
          wingValue = true;
      }
      else
      {
        wing.set_value(false);
        wingValue = false;
      } 
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) // Toggle Loader position
    {
      if(loaderValue == false)
      {
          loader.set_value(true);
          loaderValue = true;
      }
      else
      {
        loader.set_value(false);
        loaderValue = false;
      }
      
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) // Toggle Hatch value
    {
      if(middleValue == false)
      {
          middle_hatch.set_value(true);
          middleValue = true;
          pattern = ".-.-.-";
          // pattern = "";
          master.print(1, 2, "MIDDLE MODE");

      }
      else
      {
        middle_hatch.set_value(false);
        middleValue = false;
        pattern = "";
        master.clear_line(0);
      }
    }
      if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) // Toggle Low value
    {
      if(lowValue == false)
      {
          low_goal_arm.set_value(true);
          lowValue = true;
          pattern = "---.-";
          master.print(1, 2, "LOW MODE       ");

      }
      else
      {
        low_goal_arm.set_value(false);
        lowValue = false;
        pattern = "";
        master.clear_line(0);
      }

      
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
      if (mode != 5){
        mode = 5;
      } else if (mode == 5 && selected_animation < 4){
        selected_animation++;
      } else {
        mode = 1;
        selected_animation = 1;
      }
    }
    if (vibration_time_accum >= 2000){
      pros::delay(10);
      master.rumble(pattern.c_str());
      pros::delay(20);
      printf("Vibrated\n");
      vibration_time_accum = 0;
    }
    vibration_time_accum += 10;

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
