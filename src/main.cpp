/**
 * Designation: 77038V -- [[MAIN PROGRAM]]
 * Purpose: The Program that is designated to be uploaded to the
 * Team 77038V [/Voltage] Robot and is updated based on the 
 * progress of the more experimental programs.
 */

#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "Create.hpp" // Robot Setup File 


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    chassis.setPose(0, 0, 0); // set position to x:0, y:0, heading:0
    
    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // chassis.moveToPose(11.84, 45.406, 57, 5000, {.lead = 0.3,});
    // IO_velocities(-150, -150, 150);

    rightmotors.move_velocity(200);
    leftmotors.move_velocity(200);
    pros::delay(1000);
    rightmotors.move_velocity(0);
    leftmotors.move_velocity(0);

    /** 
    chassis.moveToPose(
        48,
        -24,
        90,
        2000,
        {.minSpeed=72, .earlyExitRange=8}
        // a minSpeed of 72 means that the chassis will slow down as
        // it approaches the target point, but it won't come to a full stop

        // an earlyExitRange of 8 means the movement will exit 8" away from
        // the target point
    );
    */

}

/**
 * Runs in driver control
 */
void opcontrol() {
    // loop to continuously update motors
    while (true) {
        // get left y and right x positions (get joystick positions)
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // move the robot
        chassis.tank(leftY, rightY);
        
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            MainPistons.extend();
        }
        else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            MainPistons.retract();
        }
        

        if (controller.get_digital(DIGITAL_R1)) {
            IO_velocities(150);
        }
        else if (controller.get_digital(DIGITAL_R2)) {
            IO_velocities(-150);
        }
        else {
            IO_velocities(0);
            // Required else function for when no inputs are pressed
        }




        // delay to save resources
        pros::delay(10);

        
    }
}