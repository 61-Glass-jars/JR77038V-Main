/**
 * Designation: 77038V -- [[MAIN PROGRAM]]
 * Purpose: The Program that is designated to be uploaded to the
 * Team 77038V [/Voltage] Robot and is updated based on the 
 * progress of the more experimental programs.
 * Dependants: 77038V -- [[NOTEBOOK_READABLE]]
 * Dependencies: 77038V -- [[HOME/EXPERIMENTAL]]
 *               [[LVGL_TEST]]
 */

#include "main.h"
#include "liblvgl/lvgl.h" // For LVGL Screen Interface
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller controller(pros::E_CONTROLLER_MASTER);


pros::adi::Pneumatics MainPistons('a', false);

//Motors responsible for Intake & Outtake 

pros::Motor IO4 (4, pros::MotorGearset::green);
pros::Motor IO5 (5, pros::MotorGearset::green);
pros::Motor IO7 (7, pros::MotorGearset::green);

pros::MotorGroup leftmotors({-11, -12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup rightMotors({20, 19, 18}, pros::MotorGearset::blue); // right motors use 600 RPM cartridges
// left motors on ports 11 (forwards), 12 (reversed), and 13 (reversed)
// right motors on ports 18 (reversed), 19 (forwards), and 20 (forwards)

lemlib::Drivetrain drivetrain(&leftmotors, // left motor group
                              &rightMotors, // right motor group
                              10.75, // 11 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Imu imu(10); // creates an V5 Inertial Sensors (IMU) on port 10

lemlib::OdomSensors sensors(nullptr, // no vertical tracking wheels, set to nullptr
                    nullptr, // no second vertical tracking wheel, set to nullptr
                    nullptr, // no horizontal tracking wheels, set to nullptr
                    nullptr, // no second horizontal tracking wheel, set to nullptr
                    &imu); // IMU


// lateral PID controller
lemlib::ControllerSettings lateral_controller(40, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              39, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              27, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain,
                        lateral_controller,
                        angular_controller,
                        sensors,
                        &throttle_curve, 
                        &steer_curve
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

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
            pros::lcd::print(3,"Battery Capacity: %d\n", controller.get_battery_capacity());
            pros::lcd::print(4,"Battery's Voltage: %d\n", pros::c::battery_get_voltage());
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

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
    // set position to x:0, y:0, heading:0
    // chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    // chassis.turnToHeading(90,999999999999);
    // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
    // chassis.moveToPose(20, 15, 90, 4000);
    // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
    // chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
    // cancel the movement after it has traveled 10 inches
    // chassis.waitUntil(10);
    // chassis.cancelMotion();
    // Turn to face the point x:45, y:-45. Timeout set to 1000
    // dont turn faster than 60 (out of a maximum of 127)
    // chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
    // Turn to face a direction of 90º. Timeout set to 1000
    // will always be faster than 100 (out of a maximum of 127)
    // also force it to turn clockwise, the long way around
    // chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
    // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
    // following the path with the back of the robot (forwards = false)
    // see line 116 to see how to define a path
    // chassis.follow(example_txt, 15, 4000, false);
    // wait until the chassis has traveled 10 inches. Otherwise the code directly after
    // the movement will run immediately
    // Unless its another movement, in which case it will wait
    // chassis.waitUntil(10);
    // pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
    // wait until the movement is done
    // chassis.waitUntilDone();
    // pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 */

void opcontrol() {
     // controller
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
            IO4.move_velocity(-200); // This is 200 because it's a 200rpm motor
            IO5.move_velocity(-200);
            IO7.move_velocity(-200);
            // This is mapped to Top Outtake
        }
        else if (controller.get_digital(DIGITAL_R2)) {
            IO4.move_velocity(-200); // This is 200 because it's a 200rpm motor
            IO5.move_velocity(-200);
            IO7.move_velocity(200);
            // This is mapped to Middle outake 
        }
        else if (controller.get_digital(DIGITAL_L1)) {
            IO4.move_velocity(200); // This is 200 because it's a 200rpm motor
            IO5.move_velocity(200);
            IO7.move_velocity(200);
            // This is mapped to Bottom Outtake
        }
        else {
            IO4.move_velocity(0); 
            IO5.move_velocity(0);
            IO7.move_velocity(0);
            // Required else function for when no inputs are pressed
        }




        // delay to save resources
        pros::delay(10);

        
    }
}