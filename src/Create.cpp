#include "main.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep

// Initlizing the controller object
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Creating the Pneumatics
pros::adi::Pneumatics MainPistons('a', false);

//Motors responsible for Intake & Outtake 
pros::Motor IO10 (10, pros::MotorGearset::green);
pros::Motor IO8 (8, pros::MotorGearset::green);
// Intake/Outtake motors on ports 8 and 10 (both forwards)

// Creating the components for the chassis
pros::MotorGroup leftmotors({-16, -15, -14}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup rightmotors({11, 12, 13}, pros::MotorGearset::blue); // right motors use 600 RPM cartridges
// left motors on ports 11 (forwards), 12 (reversed), and 13 (forwards)
// right motors on ports 16 (reversed), 15 (forwards), and 14 (reversed)

lemlib::Drivetrain drivetrain(&leftmotors, // left motor group
                              &rightmotors, // right motor group
                              11.75, // 11 inch track width
                              lemlib::Omniwheel::OLD_325, // using old 3.25" omnis
                              480, // drivetrain rpm is 480
                              2 // horizontal drift is 2 (for now)
);

// pros::Imu imu(10); // creates an V5 Inertial Sensors (IMU) on port 10

lemlib::OdomSensors sensors(nullptr, // no vertical tracking wheels, set to nullptr
                    nullptr, // no second vertical tracking wheel, set to nullptr
                    nullptr, // no horizontal tracking wheels, set to nullptr
                    nullptr, // no second horizontal tracking wheel, set to nullptr
                    nullptr); // IMU


// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Creating A motor Group for the outtake motors
void IO_velocities(int velocites)
{
    IO10.move_velocity(velocites);
    IO8.move_velocity(velocites);
}