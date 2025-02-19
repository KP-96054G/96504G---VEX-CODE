// void autonplus(){
//     clamp.set_value(1);
// chassis.setPose(-48, -24, 270);
// chassis.moveToPoint(-18,-24, 1500, {.forwards =  false, .maxSpeed = 80}, false);
// float v = chassis.getPose().x;
// float h = chassis.getPose().y;
// pros::delay(500);
// clamp.set_value(0);
// pros::delay(1000);
// intake.move(127);
// chassis.moveToPose(-18, -24, 175, 1500, {.maxSpeed = 80}, true);
// chassis.moveToPoint(-18, -46, 1500, {.maxSpeed = 80}, true);
// intake.move(0);
// pros::delay(1);
// intake.move(127);
// chassis.moveToPoint(-18, -24, 1500, {.maxSpeed = 80}, true);
// }

// void runIntake(){
//     for(int i = 0; i < 660; i++){
//                intake.move_velocity(1000);
//                intake.move(127);
//                pros::delay(1);

//            }
// // }

// void mtp(int k, int l, int p, bool f, int s, bool g){
//     chassis.moveToPoint(k, l, p, {.forwards = f, .maxSpeed = s}, g);
// }



        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
        //     Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        //  Arm.move(-127);
            
        // }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        //     Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        //  Arm.move(127);
            
        // }else{

        //     Arm.brake();
            
        // }

        // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        //     Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
        //     for(int i = 0; i < 160; i++){
        //         Arm.move_velocity(1000);
        //         Arm.move(-127);

        //         pros::delay(1);
        //     }
        //     Arm.brake();
        //     pros::delay(20);
        // }
// #include "main.h"
// #include "lemlib/api.hpp" // IWYU pragma: keep
// #include "lemlib/asset.hpp"
// #include "lemlib/chassis/chassis.hpp"
// #include "lemlib/chassis/trackingWheel.hpp"
// #include "lemlib/timer.hpp"
// #include "liblvgl/core/lv_obj.h"
// #include "liblvgl/extra/widgets/tileview/lv_tileview.h"
// #include "liblvgl/llemu.h"
// #include "liblvgl/llemu.hpp"
// #include "liblvgl/widgets/lv_canvas.h"
// #include "pros/abstract_motor.hpp"
// #include "pros/adi.h"
// #include "pros/adi.hpp"
// #include "pros/colors.hpp"
// #include "pros/imu.hpp"
// #include "pros/llemu.hpp"
// #include "pros/misc.h"
// #include "pros/misc.hpp"
// #include "pros/motor_group.hpp"
// #include "pros/motors.h"
// #include "pros/motors.hpp"
// //#include "okapi/api.hpp"
// #include "pros/rotation.hpp"
// #include "pros/rtos.h"
// #include "pros/rtos.hpp"
// #include <chrono>
// #include <cstddef>
// #include <ctime>
// #include <thread>
// #include <cmath>
// #include "selection.h"

// using namespace pros;
// using namespace lcd;
// using namespace pros::lcd;
// using namespace lemlib;
// using namespace c;
// using namespace competition;
// using namespace pros::competition;
// //using namespace std;

// pros::Controller controller(pros::E_CONTROLLER_MASTER);

// pros::Imu motion(21);
// pros::Rotation rotationSensor(8);

// pros::adi::DigitalOut clamp('H', false);
// pros::adi::DigitalOut doink('F', false);

// pros::Motor Arm(20);
// pros::MotorGroup intake ({10, 9}, pros::MotorGearset::blue);
// pros::MotorGroup DL({13, -5, -17}, pros::MotorGearset::blue); 
// pros::MotorGroup DR({-12, 6, 18}, pros::MotorGearset::blue);
// pros::MotorGroup Drive({-12, 6, 18, 13, -5, -17}, pros::MotorGearset::blue);


// bool clampOut = false;
// bool doinkOut = false;
// int runAuton = 0;
// bool contBuzz = false;
// int rum = 0;
// int rb1 = 0;
// int rb2 = 0;
// int rb3 = 0;
// int value = -3;
// int currState = 0;
// const int numStates = 3;
// int states[numStates] = {0, 400, 1000};
//  int target = 0;
// // const int numStates = 3;
// // //make sure these are in centidegrees (1 degree = 100 centidegrees)
// // int states[numStates] = {0, 300, 2000};
// // int currState = 0;
// // int target = 0;

// // void nextState() {
// //     currState += 1;
// //     if (currState == numStates) {
// //         currState = 0;
// //         rotation.reset_position();
// //     }
// //     target = states[currState];
// // }

// // void liftControl() {
// //     double kp = 0.005;
// //     double error = target - rotation.get_position();
// //     double velocity = kp * error;
// //     Arm.move(-velocity);
// // }
// // drivetrain settings
// lemlib::Drivetrain drivetrain(&DL, // left motor group
//                               &DR, // right motor group
//                               11.5, // 11.5 inch track width
//                               lemlib::Omniwheel::NEW_325, // using new 4" omnis

//                               450, // drivetrain rpm is 450
//                               2 // horizontal drift is 2 (for now)
// );

// // lateral PID controller
// lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               10, // derivative gain (kD)
//                                               0, // anti windup
//                                               0, // small error range, in inches
//                                               0, // small error range timeout, in milliseconds
//                                               0, // large error range, in inches
//                                               0, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );

// // angular PID controller
// lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
//                                               0, // integral gain (kI)
//                                               10, // derivative gain (kD)
//                                               0, // anti windup
//                                               0, // small error range, in degrees
//                                               0, // small error range timeout, in milliseconds
//                                               0, // large error range, in degrees
//                                               0, // large error range timeout, in milliseconds
//                                               0 // maximum acceleration (slew)
// );
// lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
//                             nullptr, // horizontal tracking wheel
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &motion // inertial sensor
// );
// // create the chassis
// lemlib::Chassis chassis(drivetrain, // drivetrain settings
//                         lateral_controller, // lateral PID settings
//                         angular_controller,
//                         sensors // angular PID settings
                        
// );

// void on_center_button() {

    
//     if(rb1 == 0){
//     static bool pressed = false;
//     pressed = !pressed;
//     if (pressed) {
//       pros::lcd::set_text(9, "RIGHT");
//       controller.print(0,1, "RIGHT SKILLS");
//       controller.rumble("_ . _");
//       value = 1;
//     } else {
//         pros::lcd::clear_line(9);
//     }
//     rb1++;
// }
    

    
// }

// void on_mid_button() {

    
//     if(rb1 == 0){
//     static bool pressed1 = false;
//     pressed1 = !pressed1;
//     if (pressed1) {
//         controller.rumble("_ . _");
//       pros::lcd::set_text(9, "LEFT");
//       controller.print(0,1, "LEFT SKILLS");
//       value = 2;
//     } else {
//       pros::lcd::clear_line(9);
//     }
//     rb1++;
//   }

// }

// void on_last_button() {

    
//     if(rb1 == 0){
//     static bool pressed2 = false;
//     pressed2 = !pressed2;
//     if (pressed2) {
//       pros::lcd::set_text(9, "SKILLS");
//       controller.print(0,1, "SKILLS AUTON");
//       value =-3;
//     } else {
//       pros::lcd::clear_line(9);
//     }
//     rb1++;
// }
    

// }

// float convert(float t){
//     t *= 1.8;
//     t += 32;
//     return t;
// }
  
// void rumbleC(std::string s){
    
    
//     controller.print(5,1, "ACTION NEEDED (", s, ")");
//     print(13, "ACTION NEEDED (", s, ")");
// }

// void nextState() { 
//     currState += 1;
//     if (currState == 2)
//      currState += 1; // Skip the 1200-degree state
//     if (currState == numStates) {
//          currState = 0; 
//          target = states [currState];
         
//          }
// }
// void liftControl() { 
//             double kp = .4;
//              double error = target - rotationSensor.get_angle();
//               double velocity = kp * error;
//             //    Arm.move(velocity);
// }

// void runBuzz(){
//             while(true){
//                 if(!pros::competition::is_connected()){
//                 if(contBuzz){
//                     controller.rumble("._.");
                    
//                 }
//             }
//         }
// }

// // initialize function. Runs on program startup
// void initialize() {
//     //runBuzz();
    
//     controller.rumble("...");
//     pros::lcd::initialize(); // initialize brain screen
//     //controller.clear();
//     //controller.print(0, 0, "Inializing...");
//     //print(0, "Inializing...");
//     chassis.calibrate(); // calibrate sensors
//     rotationSensor.reset_position();
//     rotationSensor.reset();
//     //rotationSensor.set_data_rate(15);
//     motion.reset();
//     motion.tare_rotation();
//     motion.tare_heading();
//     //pros::delay(1000);
//     //pros::lcd::clear();
//     controller.clear();
//     controller.rumble("...");


//     print(4, "RIGHT                          LEFT                        SKILLS");
//     print(5, "(RIGHT ARROW)               (LEFT ARROW)                   DEFAULt");
    
    

//     //pros::lcd::set_text_align(pros::lcd::Text_Align::CENTER);
//     pros::lcd::register_btn0_cb(on_center_button);
//     pros::lcd::register_btn1_cb(on_mid_button);
//     pros::lcd::register_btn2_cb(on_last_button);
    
    

//     pros::Task liftControlTask([]{
//         while (true) {
//             liftControl();
//             pros::delay(10);
//         }
//     });
//     pros::Task screen_task([&]() {
//         while (true) {
//             rum = 0;
//             float armt = convert(Arm.get_temperature());
//             float intaket = convert(intake.get_temperature());
//             float dlt = convert(DL.get_temperature());
//             float drt = convert(DR.get_temperature());
            
            
//             // print robot location to the brain screen
//             pros::lcd::print(0, "X: %f", (chassis.getPose().x)-48); // x
//             pros::lcd::print(1, "Y: %f", (chassis.getPose().y)-24, "            Arm Angle: %f", (rotationSensor.get_angle()/100)); // y
//             pros::lcd::print(2, "Theta: %f", (chassis.getPose().theta)+270, "        Arm Rotation: %f" , (rotationSensor.get_position()/100));
//             print(10, "Arm temp: %f", (armt), "Intake temp: %f", (intaket), "DL temp: %f", (dlt), "DR temp: %f", (drt));
//             //print(11, "IMU heading:  %f", motion.get_heading(), "IMU rotation: %f", motion.get_rotation());
//             //imu_accel_s_t accel = motion.get_accel();
//             //print(12, "IMU accel values: {x: %f, y: %f, z: %f}\n", accel.x, accel.y, accel.z);
//             if(controller.get_battery_level() <= 35){
//                 controller.print(0, 0, "CBattery -> LOW BATTERY: %f", (controller.get_battery_level()));
//                 contBuzz = false;
//             }
//             if(controller.get_battery_level() <= 15){
//                 controller.print(0, 0, "CBattery -> CHARGE NOW: %f", (controller.get_battery_level()));
//                 rumbleC("CBattery");
//                 contBuzz = true;
//             }
//             else{
//                 controller.print(0, 0, "CBattery %f", (controller.get_battery_level()));
//                 contBuzz = false;

//             }

            
//             if(Arm.is_over_temp(40)){
//                 controller.print(2,0, "ARM -> HOT: %f", (armt));
//                 contBuzz = false;
//             }
//             if(Arm.is_over_temp(50)){
//                 controller.print(2,0, "ARM -> WANRING: %f", (armt));
//                 contBuzz = false;
//             }
//             if(Arm.is_over_temp(55)){
//                 controller.print(2,0, "ARM -> COOL DOWN: %f", (armt));
//                 rumbleC("ARM");
//                 contBuzz = true;
//             }
//             if(Arm.is_over_temp(60)){
//                 controller.print(2,0, "ARM -> OVERHEATING: %f", (armt));
//                 rumbleC("ARM");
//                 contBuzz = true;
//             }
//             else{
//                 controller.print(2,0, "Arm temp: %f", (armt));
//                 contBuzz = false;
//             }

//             if(intake.is_over_temp(40)){
//                 controller.print(3,0, "Intake -> HOT: %f", (intaket));
//                 contBuzz = false;
//             }
//             if(intake.is_over_temp(50)){
//                 controller.print(3,0, "Intake -> WANRING: %f", (intaket));
//                 contBuzz = false;
//             }
//             if(intake.is_over_temp(55)){
//                 controller.print(3,0, "Intake -> COOL DOWN: %f", (intaket));
//                 rumbleC("INTAKE");
//                 contBuzz = true;
//             }
//             if(intake.is_over_temp(60)){
//                 controller.print(3,0, "Intake -> OVERHEATING: %f", (intaket));
//                 rumbleC("INTAKE");
//                 contBuzz = true;
//             }
//             else{
//                 controller.print(3,0, "Intake temp: %f", (intaket));
//                 contBuzz = false;
//             }

//             if(DL.is_over_temp(40)){
//                 controller.print(4,0, "Drive Left -> HOT: %f", (dlt));
//                 contBuzz = false;
//             }
//             if(DL.is_over_temp(50)){
//                 controller.print(4,0, "Drive Left -> WANRING: %f", (dlt));
//                 contBuzz = false;
//             }
//             if(DL.is_over_temp(55)){
//                 controller.print(4,0, "Drive Left -> COOL DOWN: %f", (dlt));
//                 rumbleC("DRIVE LEFT");
//                 contBuzz = true;
//             }
//             if(DL.is_over_temp(60)){
//                 controller.print(4,0, "Drive Left -> OVERHEATING: %f", (dlt));
//                 rumbleC("DRIVE LEFT");
//                 contBuzz = true;
//             }
//             else{
//                 controller.print(4,0, "Drive Left temp: %f", (dlt));
//                 contBuzz = false;
//             }

//             if(DR.is_over_temp(40)){
//                 controller.print(5,0, "Drive Right -> HOT: %f", (drt));
//                 contBuzz = false;
//             }
//             if(DR.is_over_temp(50)){
//                 controller.print(5,0, "Drive Right -> WANRING: %f", (drt));
//                 contBuzz = false;
//             }
//             if(DR.is_over_temp(55)){
//                 controller.print(5,0, "Drive Right -> COOL DOWN: %f", (drt));
//                 rumbleC("DRIVE RIGHT");
//                 contBuzz = true;
//             }
//             if(DR.is_over_temp(60)){
//                 controller.print(5,0, "Drive Right -> OVERHEATING: %f", (drt));
//                 rumbleC("DRIVE RIGHT");
//                 contBuzz = true;

//             }
//             else{
//                 controller.print(5,0, "Drive Right temp: %f", (dlt));
//                 contBuzz = false;
//             }


//             controller.print(6,0, "Arm Angle: %f", (rotationSensor.get_angle()));
//             controller.print(7, 0, "IMU heading:  %f", motion.get_heading());
//             controller.print(8, 0, "IMU Rotation:  %f", motion.get_rotation());
//             controller.print(1,0, "CBattery Capacity: %f", controller.get_battery_capacity());
//             controller.print(0,2, "Battery Capacity: %f", (battery_get_capacity()));
//             controller.print(1,2, "Battery Temp: %f", (battery_get_temperature()));
//             controller.print(2,2, "Battery Current: %f", (battery_get_current()));
//             controller.print(3,2, "Battery Voltage: %f", (battery_get_voltage()));
//             imu_accel_s_t accel2 = motion.get_accel();
//             controller.print(5, 2, ": {x: %f, y: %f, z: %f}\n", accel2.x, accel2.y, accel2.z);
//             if (pros::competition::is_connected()) {
//                 controller.print(3, 1, "Connected to VEXnet");
//                 controller.rumble(". .");
//             }
            
//             if(pros::competition::is_autonomous())    {
//                 controller.print(4, 1, "Running Auton");
//                 controller.rumble("_ _");
//             }        

//             if(rb1 == 0){
//             if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
//                 on_center_button();
                
                
//             }
//             else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
//                 on_mid_button();
                
//             }
//             else{
//                 pros::lcd::print(9, "Currently set to skills");
//                 controller.print(0,1, "Auton set to skills press <- or -> to change");
//             }
//         }
            
            
            


//             pros::delay(25);
            
//         }
//     });
// }

// //pros::Controller controller(pros::E_CONTROLLER_MASTER);
// /**
//  * Runs initialization code. This occurs as soon as the program is started.
//  *
//  * All other competition modes are blocked by initialize; it is recommended
//  * to keep execution time for this mode under a few seconds.
//  */
// /**
//  * Runs while the robot is in the disabled state of Field Management System or
//  * the VEX Competition Switch, following either autonomous or opcontrol. When
//  * the robot is enabled, this task will exit.
//  */
// void disabled() {
    
// }

// /**
//  * Runs after initialize(), and before autonomous when connected to the Field
//  * Management System or the VEX Competition Switch. This is intended for
//  * competition-specific initialization routines, such as an autonomous selector
//  * on the LCD.
//  *
//  * This task will exit when the robot is enabled and autonomous or opcontrol
//  * starts.
//  */
// void competition_initialize() {
//     // if(rb1 == 0){
//     //     if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
//     //         on_center_button();
            
            
//     //     }
//     //     else if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
//     //         on_mid_button();
            
//     //     }
//     //     else{
//     //         pros::lcd::print(9, "Currently set to skills");
//     //         controller.print(0,1, "Auton set to skills press <- or -> to change");
//     //     }
//     // }
// }

// void d(int k){
//     delay(k);
// }


// void reverseAuton(int k){
//     for(int i = 0; i < k; i++){
//         delay(1);
//     }
//     intake.move(0);
//     for(int i = 0; i < 100; i++){
//         intake.move(-127);
//     }
// } 


// //PROG SKILLS
// void progSkills() {
//     clamp.set_value(1);
//     chassis.setPose(-62, 0, 90);
//     intake.move(127);
//     delay(500);
//     chassis.moveToPoint(-55, 0, 1500, {.maxSpeed = 70}, false);
//     chassis.moveToPoint(-50, -27, 1700 , {.forwards = false, .maxSpeed = 60}, false);
//     delay(150);
//     clamp.set_value(0);
//     delay(200);
//     chassis.moveToPoint(-24, -24, 1700, {.maxSpeed = 70}, true);
//     delay(200);
//     chassis.turnToHeading(180,500);
//     chassis.moveToPoint(-24, -48, 1700, {.maxSpeed = 70}, true);
//     delay(200);
//     chassis.turnToHeading(270, 500);
//     delay(200); 
//     chassis.moveToPoint(-60, -48, 3500, {.maxSpeed = 50}, true);
//     delay(500);
//     chassis.moveToPoint(-41, -62, 1700, {.maxSpeed = 70}, true);
//     delay(200);
//     chassis.moveToPoint(-66, -63, 1700, {.forwards = false, .maxSpeed = 70}, false);
//     intake.move(-100);
//     delay(200);
//     intake.move(0);
//     clamp.set_value(1);
//     chassis.moveToPoint(-48, -46, 3000, {.maxSpeed = 70}, false);
//     intake.move(127);
//     chassis.moveToPoint(-49, 26, 3500, {.forwards = false, .maxSpeed = 50}, false);
//     delay(200);
//     intake.move(0);
//     delay(500);
//     clamp.set_value(0);
//     delay(500);
//     intake.move(127);
//     chassis.moveToPoint(-24, 24, 1500, {.maxSpeed = 70}, true);
//     chassis.turnToHeading(0, 500);
//     chassis.moveToPoint(-24, 51, 1500, {.maxSpeed = 70}, true);
//     chassis.turnToHeading(270, 500);
//     chassis.moveToPoint(-62, 51, 2700, {.maxSpeed = 70}, true);
//     chassis.moveToPoint(-43, 61, 2200, {.maxSpeed = 70}, true);
//     chassis.moveToPoint(-65, 60, 1500, {.forwards = false, .maxSpeed = 70}, false);
//     intake.move(-127);
//     clamp.set_value(1);
//     delay(100);
//     intake.move(0);
//     chassis.moveToPoint(56, 6, 3500, {.forwards = true, .maxSpeed = 70}, false);
//     chassis.turnToHeading(180, 500);
//     chassis.moveToPoint(63,57, 3500, {.forwards = false, .maxSpeed = 90}, false);
//     chassis.turnToHeading(0, 500);
//     chassis.moveToPoint(63,-57, 3500, {.forwards = false, .maxSpeed = 90}, false);
// }

// void ourRight() {
  
//     clamp.set_value(1);
//     chassis.setPose(-64, -25, 270);
//     chassis.moveToPoint(-24, -25, 1500, {.forwards = false, .maxSpeed = 60}, false);
//     clamp.set_value(0);
//     delay(300);
//     intake.move(127);
//     chassis.moveToPoint(-24, -45, 1500, {.maxSpeed = 60}, true);
//     delay(1000);
//     intake.move(0);

//     chassis.moveToPoint(-24, -40, 1500, {.maxSpeed = 60}, true);
//     intake.move(127);
//     chassis.moveToPoint(-40, -53, 1500, {.forwards = false, .maxSpeed = 70}, false);
//     clamp.set_value(1);
//     chassis.moveToPoint(-25, -45, 1500, {.forwards = false, .maxSpeed = 70}, false); 
// }

// void ourleft() {
  
//     clamp.set_value(1);
//     chassis.setPose(-64, 25, 270);
//     chassis.moveToPoint(-24, 25, 1500, {.forwards = false, .maxSpeed = 60}, false);
//     clamp.set_value(0);
//     delay(300);
//     intake.move(127);
//     chassis.moveToPoint(-24, 45, 1500, {.maxSpeed = 60}, true);
//     delay(1000);
//     intake.move(0);

//     chassis.moveToPoint(-24, 40, 1500, {.maxSpeed = 60}, true);
//     intake.move(127);
//     chassis.moveToPoint(-40, 53, 1500, {.forwards = false, .maxSpeed = 70}, false);
//     clamp.set_value(1);
//     chassis.moveToPoint(-25, 45, 1500, {.forwards = false, .maxSpeed = 70}, false); 

// }


// void autonomous() {

//     if(value== -3){
//         progSkills();
//     }
//     else if(value == 1){
//         ourRight();
//     }
//     else if(value == 2){
//         ourleft();
//     }
//     else{
//         progSkills();
//     }
// }
// /**
//  * Runs the operator control code. This function will be started in its own task
//  * with the default priority and stack size whenever the robot is enabled via
//  * the Field Management System or the VEX Competition Switch in the operator
//  * control mode
//  * If no competition control is connected, this function will run immediately
//  * following initialize().
//  *
//  * If the robot is disabled or communications is lost, the
//  * operator control task will be stopped. Re-enabling the robot will restart the
//  * task, not resume it from where it left off.
//  */


// void opcontrol() {
//     // controller
//     // loop to continuously update motors
//     Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);

//     //!Arm.move_velocity(0);
//     // bool t = true;
//     // bool e = true;
//     // bool r = true;
//     // int c = 0;

//     while (true) {

//         pros::lcd::print(7, "Buttons Bitmap: %d\n", pros::lcd::read_buttons());
        
        
//         // get joystick positions
//         int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
//         int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
//         // move the chassis with curvature drive
//         chassis.arcade(leftY, rightX);


//         if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
//             clampOut = !clampOut;
//             clamp.set_value(clampOut);
//         }

//         // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
//         //     doinkOut = !doinkOut;
//         //     doink.set_value(doinkOut);
            
//         // }

//         if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
//             Arm.move(127);
//         }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
//             Arm.move(-127);
//         }else{
//             Arm.brake();
//         }
        
//         if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
//             for(int i = 0; i < 186; i++){
//                 Arm.move_velocity(1000);
//                 Arm.move(-127);

//                 pros::delay(1);
//                 Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
//             }
//             Arm.brake();
//             pros::delay(20);
//             Arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);

//         }

        
//         if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
//             intake.move(127);
//         }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
//             intake.move(-127);
//         }else{
//             intake.move(0);
//         }

//         if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && runAuton == 0){
//             runAuton++;
//             autonomous();
//         }

//         // delay to save resources
//         pros::delay(10);
//     }

// }


