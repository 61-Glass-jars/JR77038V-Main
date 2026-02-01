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

void Left_side() {
    // function for left side autonomous
    // Move between Long goal and Loader
    chassis.moveToPoint(0, 18.248, 2000);
    chassis.turnToHeading(270, 2000 );
    IO_velocities(200,-300,200);
    Hood.retract();

    // Collect Octoballs from Bottom Right Pile
    chassis.moveToPose(-6.292, 46.982, 0, 3000, {.lead=0.9});
    chassis.turnToHeading(177, 1500, {.maxSpeed = 75});
    
    //postiing bot between loader and long goal
    chassis.moveToPose(-29, 14.53, 270, 3000., {.lead=0.575, });
    chassis.waitUntilDone();
    chassis.setPose(-30.7, 14.53, 270);
    chassis.turnToHeading(180, 2000);
    // scraperPistion.toggle();

    // Collect Octoballs from Loader
    // chassis.moveToPoint(-30.7,  6, 3000);
    // chassis.waitUntilDone();
    // pros::delay(200);

    // Output Octoballs into Long Goal
    chassis.moveToPoint(-30.7, 31, 5000,{.forwards = false});
    chassis.waitUntilDone();
    Hood.extend();
}

void Right_side() {
    // function for right side autonomous
    // Move between Long goal and Loader
    chassis.moveToPoint(0, 18.248, 5000);
    chassis.turnToHeading(90, 3000);
    IO_velocities(200,-300,200);
    Hood.retract();

    // Collect Octoballs from Bottom Right Pile
    chassis.moveToPose(6.292, 46.982, 0, 5000, {.horizontalDrift = 2,.lead=0.9});
    chassis.turnToHeading(180, 3000);
    
    //postiing bot between loader and long goal
    chassis.moveToPose(30.7, 14.531, 90, 5000., {.horizontalDrift = 2,.lead=0.6});
    chassis.turnToHeading(180, 3000);

    // Collect Octoballs from Loader
    // chassis.moveToPoint(-30.7,  6, 3000);
    // chassis.waitUntilDone();
    // pros::delay(200);
    
    // Output Octoballs into Long Goal
    chassis.moveToPoint(30.7, 31, 5000,{.forwards = false});
    chassis.waitUntilDone();
    Hood.extend();
}

void Skills() {
    // Move between Long goal and Loader 
    chassis.moveToPoint(0, 47, 5000, {.maxSpeed=100});
    chassis.turnToHeading(90, 3000);
    IO_velocities(200,-300,200);
    scraperPistion.toggle();
    Hood.retract();

    // Collect Octoballs from Loader
    chassis.moveToPoint(10, 47, 5000);
    chassis.waitUntilDone();
    pros::delay(500);

    // Output Octoballs into Long Goal
    chassis.moveToPoint(-15, 47, 5000,{.forwards = false});
    chassis.waitUntilDone();
    Hood.extend();
    pros::delay(1000);
    scraperPistion.toggle();
    Hood.retract();

    // Position to Collect Octoballs from Botom right Center Field
    chassis.moveToPose(0, 23, 180, 5000, {.horizontalDrift = 2,.lead=0.8});
    chassis.turnToHeading(270, 3000);

    // Collect Octoballs from Bottom right Center Field
    chassis.moveToPoint(-27, 23, 5000);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 3000);


    // deposit in low goal
    chassis.moveToPose(-33.28, 13.121, 225, 5000, {.horizontalDrift = 2,.lead=0.65});
    chassis.waitUntilDone();
    IO_velocities(-200,300,-200);

}

void Var_15PointSkills() {
    chassis.moveToPoint(0, 10, 3000, {.minSpeed=100});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 20, 3000, {.minSpeed=100});
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
    Left_side();
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
        
        // Pneumatics Control
        
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            scraperPistion.toggle();
        }
        


        // Intake & Outtake Control
        if (controller.get_digital(DIGITAL_R1)) {
            IO_velocities(200,-300,200);
            Hood.retract();
            // Intake with Hood retracted
        }
        else if (controller.get_digital(DIGITAL_R2)) {
            IO_velocities(-200,300,-200);
            // Bottom outtake
        }
        else if (controller.get_digital(DIGITAL_L1)) {
            IO_velocities(200,-300,200);
            Hood.extend();
            // Intake with Hood extended
        }
        else if (controller.get_digital(DIGITAL_L2)) {
            IO_velocities(-200,-300,200);
            Hood.retract();
            // Middle outtake

        }
        else {
            IO_velocities(0,0,0);
            // Required else function for when no inputs are pressed
        }




        // delay to save resources
        pros::delay(10);

        
    }
}