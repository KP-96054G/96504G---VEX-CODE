#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "lemlib/timer.hpp"
#include "liblvgl/core/lv_obj.h"
#include "liblvgl/extra/widgets/tileview/lv_tileview.h"
#include "liblvgl/llemu.h"
#include "liblvgl/llemu.hpp"
#include "liblvgl/widgets/lv_canvas.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/colors.h"
#include "pros/colors.hpp"
#include "pros/imu.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
//#include "okapi/api.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include <chrono>
#include <cstddef>
#include <ctime>
#include <thread>
#include <cmath>
#include "selection.h"

using namespace pros;
using namespace lcd;
using namespace pros::lcd;
using namespace lemlib;
using namespace c;
using namespace competition;
using namespace pros::competition;
using namespace screen;
using namespace pros::screen;
//using namespace std;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::Imu motion(21);
pros::Rotation rotationSensor(8);

pros::adi::DigitalOut clamp('H', false);
pros::adi::DigitalOut doink('F', false);

pros::Motor Arm(20);
pros::MotorGroup intake ({10, 9}, pros::MotorGearset::blue);
pros::MotorGroup DL({13, -5, -17}, pros::MotorGearset::blue); 
pros::MotorGroup DR({-12, 6, 18}, pros::MotorGearset::blue);
pros::MotorGroup Drive({-12, 6, 18, 13, -5, -17}, pros::MotorGearset::blue);
Motor one(13);
Motor two(-5);
Motor three(-17);
Motor four(-12);
Motor five(6);
Motor sixe(18);

bool clampOut = false;
bool doinkOut = false;
int runAuton = 0;
bool contBuzz = false;
int rum = 0;
int rb1 = 0;
int rb2 = 0;
int rb3 = 0;
int value = -3;
int currState = 0;
bool earase = false;
bool oner = false;
bool twor = false;
const int numStates = 3;
int states[numStates] = {0, 400, 1000};
 int target = 0;
// const int numStates = 3;
// //make sure these are in centidegrees (1 degree = 100 centidegrees)
// int states[numStates] = {0, 300, 2000};
// int currState = 0;
// int target = 0;

// void nextState() {
//     currState += 1;
//     if (currState == numStates) {
//         currState = 0;
//         rotation.reset_position();
//     }
//     target = states[currState];
// }

// void liftControl() {
//     double kp = 0.005;
//     double error = target - rotation.get_position();
//     double velocity = kp * error;
//     Arm.move(-velocity);
// }
// drivetrain settings
lemlib::Drivetrain drivetrain(&DL, // left motor group
                              &DR, // right motor group
                              11.5, // 11.5 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis

                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &motion // inertial sensor
);
// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller,
                        sensors // angular PID settings
                        
);

  



float convert(float o, float t, float th){
    o = o + t + th;
    return o/3;
}
  


void nextState() { 
    currState += 1;
    if (currState == 2)
     currState += 1; // Skip the 1200-degree state
    if (currState == numStates) {
         currState = 0; 
         target = states [currState];
         
         }
}
void liftControl() { 
            double kp = .4;
             double error = target - rotationSensor.get_angle();
              double velocity = kp * error;
            //    Arm.move(velocity);
}

void runBuzz(){
            while(true){
                if(!pros::competition::is_connected()){
                if(contBuzz){
                    controller.rumble("._.");
                    
                }
            }
        }
}

void changePixel() {
    pros::screen_touch_status_s_t status = pros::screen::touch_status();
    //pros::screen::draw_pixel(status.x, status.y);
    if(status.y < 120){
        controller.print(0,0, "RIGHT");
        controller.rumble("...");
        value = 1;
        rb1 = 1;
    }
    if(status.y >= 120){
        controller.print(0,0, "LEFT");
        controller.rumble("___");
        value = 2;
        rb1 = 1;
    }
    
 }

// initialize function. Runs on program startup
void initialize() {

    chassis.calibrate(); // calibrate sensors
    rotationSensor.reset_position();
    rotationSensor.reset();
    //rotationSensor.set_data_rate(15);
    motion.reset();
    motion.tare_rotation();
    motion.tare_heading();
    int c = 0;
    //runBuzz();
    controller.rumble("______.");
    


    pros::Task liftControlTask([]{
        while (true) {
            liftControl();
            pros::delay(10);
        }
    });


    pros::Task screen_task([&]() {
        while(true){
            
            // if(c == 0){
            //     float d = convert(one.get_temperature(), two.get_temperature(), three.get_temperature());
            //     controller.print(0, 0, "DR: %f", d);
            //     c++;
            // }
            // else if(c == 1){
            //     float d = convert(four.get_temperature(), five.get_temperature(), sixe.get_temperature());
            //     controller.print(0, 0, "DL: %f", d);
            //     c = 0;
            // }
            // else{
            //     controller.print(0, 0, "ERROR");
            // }
            //delay(3000);
            if(rb1 == 0){
                set_pen(COLOR_ORANGE_RED);
                pros::screen::draw_rect(1,1,479,119);
                set_pen(COLOR_ORANGE_RED);
                pros::screen::fill_rect(1,1,479,119);
                set_pen(COLOR_BLUE_VIOLET);
                pros::screen::draw_rect(1,201,479,239);
                set_pen(COLOR_BLUE_VIOLET);
                pros::screen::fill_rect(1,201,479,239);
                set_pen(COLOR_GOLD);
                pros::screen::print(pros::E_TEXT_LARGE,100, 50, "RIGHT");
                pros::screen::print(pros::E_TEXT_LARGE,100, 170, "LEFT");
                pros::screen::touch_callback(changePixel, TOUCH_PRESSED);
            
            }
            else if(rb1 == 1 && earase == false){
                pros::screen::set_eraser(COLOR_ALICE_BLUE);
                pros::screen::erase();
                earase = true;
            }
            else{
                set_pen(COLOR_BLACK);
                pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Arm Angle: %f", rotationSensor.get_angle(), ", Arm Rotation: %f", rotationSensor.get_position(), ",Arm Velo: %f", rotationSensor.get_velocity());
                imu_accel_s_t accel2 = motion.get_accel();
                pros::screen::print(E_TEXT_MEDIUM, 3, "Imu Rotation:  %f", motion.get_rotation(), ", Imu Heading: %f", motion.get_heading(), ", Imu accel: {x: %f, y: %f, z: %f}\n", accel2.x, accel2.y, accel2.z);
                pros::screen::print(pros::E_TEXT_SMALL, 4, "Arm Temp: %f, Intake Temp: %f, L1: %f, L2: %f, L3: %f, R1: %f, R2: %f, R3: %f", Arm.get_temperature(), intake.get_temperature(), one.get_temperature(), two.get_temperature(), three.get_temperature(), four.get_temperature(), five.get_temperature(), sixe.get_temperature());
            }
            if (pros::competition::is_connected()) {
                if(oner == false){
                    controller.rumble(". .");
                    oner = true;
                }
            }
                        
            if(pros::competition::is_autonomous())    {
                if(twor == false){
                    controller.rumble("_ _");
                    twor = true;
            }
            
        }
            delay(500);
            if(rb1 == 1 && earase){
                pros::screen::erase();
            }    
        }
            
            

       
    });
}

//pros::Controller controller(pros::E_CONTROLLER_MASTER);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    // if(rb1 == 0){
    //     if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
    //         on_center_button();
            
            
    //     }
    //     else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
    //         on_mid_button();
            
    //     }
    //     else{
    //         pros::lcd::print(9, "Currently set to skills");
    //         controller.print(0,1, "Auton set to skills press <- or -> to change");
    //     }
    // }
}

void d(int k){
    delay(k);
}


void reverseAuton(int k){
    for(int i = 0; i < k; i++){
        delay(1);
    }
    intake.move(0);
    for(int i = 0; i < 100; i++){
        intake.move(-127);
    }
} 


//PROG SKILLS
void progSkills() {
    clamp.set_value(1);
    chassis.setPose(-62, 0, 90);
    intake.move(127);
    delay(500);
    chassis.moveToPoint(-55, 0, 1500, {.maxSpeed = 70}, false);
    chassis.moveToPoint(-50, -27, 1700 , {.forwards = false, .maxSpeed = 60}, false);
    delay(150);
    clamp.set_value(0);
    delay(200);
    chassis.moveToPoint(-24, -24, 1700, {.maxSpeed = 70}, true);
    delay(200);
    chassis.turnToHeading(180,500);
    chassis.moveToPoint(-24, -48, 1700, {.maxSpeed = 70}, true);
    delay(200);
    chassis.turnToHeading(270, 500);
    delay(200); 
    chassis.moveToPoint(-60, -48, 3500, {.maxSpeed = 50}, true);
    delay(500);
    chassis.moveToPoint(-41, -62, 1700, {.maxSpeed = 70}, true);
    delay(200);
    chassis.moveToPoint(-66, -63, 1700, {.forwards = false, .maxSpeed = 70}, false);
    intake.move(-100);
    delay(200);
    intake.move(0);
    clamp.set_value(1);
    chassis.moveToPoint(-48, -46, 3000, {.maxSpeed = 70}, false);
    intake.move(127);
    chassis.moveToPoint(-49, 26, 3500, {.forwards = false, .maxSpeed = 50}, false);
    delay(200);
    intake.move(0);
    delay(500);
    clamp.set_value(0);
    delay(500);
    intake.move(127);
    chassis.moveToPoint(-24, 24, 1500, {.maxSpeed = 70}, true);
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(-24, 51, 1500, {.maxSpeed = 70}, true);
    chassis.turnToHeading(270, 500);
    chassis.moveToPoint(-62, 51, 2700, {.maxSpeed = 70}, true);
    chassis.moveToPoint(-43, 61, 2200, {.maxSpeed = 70}, true);
    chassis.moveToPoint(-65, 60, 1500, {.forwards = false, .maxSpeed = 70}, false);
    intake.move(-127);
    clamp.set_value(1);
    delay(100);
    intake.move(0);
    chassis.moveToPoint(56, 6, 3500, {.forwards = true, .maxSpeed = 70}, false);
    chassis.turnToHeading(180, 500);
    chassis.moveToPoint(63,57, 3500, {.forwards = false, .maxSpeed = 90}, false);
    chassis.turnToHeading(0, 500);
    chassis.moveToPoint(63,-57, 3500, {.forwards = false, .maxSpeed = 90}, false);
}

void ourRight() {
  
    clamp.set_value(1);
    chassis.setPose(-64, -25, 270);
    chassis.moveToPoint(-24, -25, 1500, {.forwards = false, .maxSpeed = 60}, false);
    clamp.set_value(0);
    delay(300);
    intake.move(127);
    chassis.moveToPoint(-24, -45, 1500, {.maxSpeed = 60}, true);
    delay(1000);
    intake.move(0);

    chassis.moveToPoint(-24, -40, 1500, {.maxSpeed = 60}, true);
    intake.move(127);
    chassis.moveToPoint(-40, -53, 1500, {.forwards = false, .maxSpeed = 70}, false);
    clamp.set_value(1);
    chassis.moveToPoint(-25, -45, 1500, {.forwards = false, .maxSpeed = 70}, false); 
}

void ourleft() {
  
    clamp.set_value(1);
    chassis.setPose(-64, 25, 270);
    chassis.moveToPoint(-24, 25, 1500, {.forwards = false, .maxSpeed = 60}, false);
    clamp.set_value(0);
    delay(300);
    intake.move(127);
    chassis.moveToPoint(-24, 45, 1500, {.maxSpeed = 60}, true);
    delay(1000);
    intake.move(0);

    chassis.moveToPoint(-24, 40, 1500, {.maxSpeed = 60}, true);
    intake.move(127);
    chassis.moveToPoint(-40, 53, 1500, {.forwards = false, .maxSpeed = 70}, false);
    clamp.set_value(1);
    chassis.moveToPoint(-25, 45, 1500, {.forwards = false, .maxSpeed = 70}, false); 

}


void autonomous() {

    if(value== -3){
        progSkills();
    }
    else if(value == 1){
        ourRight();
    }
    else if(value == 2){
        ourleft();
    }
    else{
        progSkills();
    }
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
    // controller
    // loop to continuously update motors
    Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    //!Arm.move_velocity(0);
    // bool t = true;
    // bool e = true;
    // bool r = true;
    // int c = 0;

    while (true) {

        
        
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);


        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            clampOut = !clampOut;
            clamp.set_value(clampOut);
        }

        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
        //     doinkOut = !doinkOut;
        //     doink.set_value(doinkOut);
            
        // }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            if(rb1 == 0){
                controller.print(0,0, "RIGHT");
                controller.rumble("...");
                value = 1;
                rb1 = 1;
            }
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            if(rb1 == 0){
                controller.print(0,0, "LEFT");
                controller.rumble("___");
                value = 2;
                rb1 = 1;
            }
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            Arm.move(127);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            Arm.move(-127);
        }else{
            Arm.brake();
        }
        
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            for(int i = 0; i < 186; i++){
                Arm.move_velocity(1000);
                Arm.move(-127);

                pros::delay(1);
                Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
            }
            Arm.brake();
            pros::delay(20);
            Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);

        }

        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            intake.move(127);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            intake.move(-127);
        }else{
            intake.move(0);
        }

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && runAuton == 0){
            runAuton++;
            autonomous();
        }

        // delay to save resources
        pros::delay(10);
    }

}
