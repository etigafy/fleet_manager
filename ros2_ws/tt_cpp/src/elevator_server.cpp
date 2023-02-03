// #define DEBUG_MSGS
// #define SAFETY_FEATURES_ENABLE

#include "rclcpp/rclcpp.hpp"
#include "h_interfaces/srv/elevator_pos.hpp"

#include <stdio.h>
#include <gpiod.h>
#include <thread>
#include <chrono>
#include <cstring>
#include <memory>
#include <math.h>

// pinout
#define STEP_PIN 9
#define DIR_PIN 10
#define STOP_PIN 11
#define ENABLE_PIN 12

// stepper settings/geometry
#define INVERT_DIRECTION 0
#define HOMING_DIRECTION 0
#define MICROSTEPPING 8
#define MM_PER_REV 8
#define STEP_ANGLE 1.8 // degrees

// actuation limits
#define MAX_LIN_VEL (double)12.5 // mm/s
#define LIN_ACCEL (double)15 // mm/s^2
#define Z_AXIS_LOWER_LIMIT 0 // mm
#define Z_AXIS_UPPER_LIMIT 300 // mm


// elevator vars
float zPos = 0;
bool enableEndstop = false;

// init GPIO vars
struct gpiod_chip *chip;
struct gpiod_line *line_step, *line_dir, *line_stop, *line_enable;

// functions
void moveElevator(const std::shared_ptr<h_interfaces::srv::ElevatorPos::Request> request,
          std::shared_ptr<h_interfaces::srv::ElevatorPos::Response> response);
int init_GPIO();
void shutdown_GPIO();
void homeElevator();
void enableElevator();
void disableElevator();
void moveTo(double dest_point, double lin_vel = MAX_LIN_VEL, double lin_accel = LIN_ACCEL);


int main(int argc, char **argv)
{
    if(init_GPIO() < 0) return -1;

    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("elevator_server");

    rclcpp::Service<h_interfaces::srv::ElevatorPos>::SharedPtr service =
        node->create_service<h_interfaces::srv::ElevatorPos>("elevator_service", &moveElevator);

    RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "Elevator server started.");
    
    // home
    homeElevator();

    rclcpp::spin(node);

    rclcpp::shutdown();
    shutdown_GPIO();
}

// --------------------------------------------------------------------------------------
int init_GPIO()
{
    int err;

    // grab gpio chip
    chip = gpiod_chip_open("/dev/gpiochip0");

    if(!chip)
    {
        perror("error - gpiod_chip_open\n");
        return -1;
    }

    line_step = gpiod_chip_get_line(chip, STEP_PIN);
    line_dir = gpiod_chip_get_line(chip, DIR_PIN);
    line_stop = gpiod_chip_get_line(chip, STOP_PIN);
    line_enable = gpiod_chip_get_line(chip, ENABLE_PIN);

    if(!line_step || !line_dir || !line_stop || !line_enable)
    {
        perror("error - gpiod_chip_get_lines!\n");
        return -1;
    }

    // configure pins
    struct gpiod_line_request_config configOutput;
    memset(&configOutput, 0, sizeof(configOutput));
    configOutput.consumer = "stepper";
    configOutput.request_type = GPIOD_LINE_REQUEST_DIRECTION_OUTPUT;
    configOutput.flags = 0;

    struct gpiod_line_request_config configInput;
    memset(&configInput, 0, sizeof(configInput));
    configInput.consumer = "endStop";
    configInput.request_type = GPIOD_LINE_REQUEST_DIRECTION_INPUT;
    configInput.flags = 0;

    // take ownershiop of pins
    err = gpiod_line_request(line_step, &configOutput, 0);
    if(err < 0)
    {
        perror("error - gpiod_line_request: line_step!\n");
        return -1;
    }

    err = gpiod_line_request(line_dir, &configOutput, 0);
    if(err < 0)
    {
        perror("error - gpiod_line_request: line_dir!\n");
        return -1;
    }

    err = gpiod_line_request(line_stop, &configInput, 0);
    if(err < 0)
    {
        perror("error - gpiod_line_request: line_stop!\n");
        return -1;
    }

    err = gpiod_line_request(line_enable, &configOutput, 0);
    if(err < 0)
    {
        perror("error - gpiod_line_request: line_enable!\n");
        return -1;
    }

    return 0;
}

// --------------------------------------------------------------------------------------
void shutdown_GPIO()
{
    // cleanup
    gpiod_chip_close(chip);
}

// --------------------------------------------------------------------------------------
void homeElevator()
{
    // enable elevator
    enableElevator();

    while(gpiod_line_get_value(line_stop) == 0)
    {
        RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "error: Check homing endstop...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }

    RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "Homing...");

    int direction;
    if(HOMING_DIRECTION == 0) direction = 1;
    else direction = -1;

    enableEndstop = true;
    moveTo(direction * (-Z_AXIS_UPPER_LIMIT), 5, 5);
    zPos = 0;
    moveTo(direction * 3, 5, 5);
    zPos = 0;

    // enableEndstop = true;
    // moveTo(direction * (-5), 5, 10);
    // zPos = 0;
    // moveTo(direction * 3, 10, 10);
    // zPos = 0;

    // disable endstop in case it was not used during second homing round
    enableEndstop = false;
    RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "Homing complete!");

    // disable stepper
    disableElevator();
}

// --------------------------------------------------------------------------------------
void enableElevator()
{
    gpiod_line_set_value(line_enable, 0);
}

void disableElevator()
{
    gpiod_line_set_value(line_enable, 1);
}

// --------------------------------------------------------------------------------------
void moveElevator(const std::shared_ptr<h_interfaces::srv::ElevatorPos::Request> request,
          std::shared_ptr<h_interfaces::srv::ElevatorPos::Response> response)
{
    // enable stepper
    enableElevator();

    double goal_pos = request->position;
    RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "Position request: %f mm -> %f mm", zPos, request->position);

    #ifdef SAFETY_FEATURES_ENABLE
        // check axis limits
        if(request->position > Z_AXIS_UPPER_LIMIT || request->position < Z_AXIS_LOWER_LIMIT)
        {
            RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "Requested motion exceeds axis limits - min: %d, max: %d", Z_AXIS_UPPER_LIMIT, Z_AXIS_LOWER_LIMIT);
            response->status = false;
            return;
        }
    #endif

    // move axis
    if(request->position == 0)
    {
        moveTo(5, request->velocity, request->acceleration);
        homeElevator();
    }
    else
    {
        moveTo(request->position, request->velocity, request->acceleration);
    }

    response->status = true;

    // update zPos
    zPos = request->position;

    // disable stepper if it is at the bottom
    if(zPos == 0) disableElevator();

    RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "Elevator motion complete: %f mm", zPos);
}

// --------------------------------------------------------------------------------------
void moveTo(double dest_point, double lin_vel, double lin_accel)
{
    lin_vel /= MM_PER_REV;
    lin_accel /= MM_PER_REV;

    #ifdef SAFETY_FEATURES_ENABLE
        // check vel and acc request
        if(lin_vel <= 0 || lin_vel > MAX_LIN_VEL)
        {
            RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "Invalid velocity request (%f mm/s) - using: %f mm/s.", lin_vel, MAX_LIN_VEL);
            lin_vel = MAX_LIN_VEL;
        }
        if(lin_accel <= 0 || lin_accel > LIN_ACCEL)
        {
            RCLCPP_INFO(rclcpp::get_logger("Elevator Server"), "Invalid acceleration request (%f mm/s^2) - using: %f mm/s^2.", lin_accel, LIN_ACCEL);
            lin_accel = LIN_ACCEL;
        }
    #endif

    size_t steps_per_rev = 360 / STEP_ANGLE * MICROSTEPPING;

    // compute steps required
    double distance = dest_point - zPos; // in mm
    size_t stepsRequired = (size_t) abs(distance * steps_per_rev / MM_PER_REV);

    #ifdef DEBUG_MSGS
        printf("steps: %ld\n", stepsRequired); 
    #endif

    // set direction pin and invert if flag invert_bit is set
    bool direction = std::signbit(distance) ^ INVERT_DIRECTION;

    #ifdef DEBUG_MSGS 
        printf("dir: %d\n", direction);
    #endif

    if(direction)
    {
        gpiod_line_set_value(line_dir, 0);
    }
    else
    {
        gpiod_line_set_value(line_dir, 1);
    }

    // compute velocity trajectory
    double alpha = 2 * M_PI / steps_per_rev;
    double maxAngularVel = lin_vel * 2 * M_PI;
    double angularAccel = lin_accel * 2 * M_PI;

    long maxInterval = 1 / (steps_per_rev * maxAngularVel / (2 * M_PI)) * 1000000; // *1000000 for s to us conversion
    size_t rampUpStep = (size_t) pow(maxAngularVel, 2) / (2 * alpha * angularAccel) + 0.5;
    size_t rampDownStep;

    if(stepsRequired / 2 < rampUpStep)
    {
        rampUpStep = stepsRequired / 2;
        rampDownStep = rampUpStep;
    }
    else
    {
        rampDownStep = stepsRequired - rampUpStep;
    }

    double stepInterval = 0.676 * sqrt(2 * alpha / angularAccel) * 1000000; // *1000000 for s to us conversion

    size_t stepCounter = 0;

    // set initial step timer
    auto stepTimer = std::chrono::high_resolution_clock::now() + std::chrono::microseconds((long int)stepInterval);

    while(1)
    {   
        // if endstop is used, check state
        if(enableEndstop)
        {            
            if(gpiod_line_get_value(line_stop) == 0)
            {
                printf("Endstop triggered!\n");
                enableEndstop = false;
                return;
            }
        }

        // step once
        gpiod_line_set_value(line_step, 1);
        gpiod_line_set_value(line_step, 0);
        stepCounter++;

        // break condition
        if(stepCounter >= stepsRequired) break;

        while(std::chrono::high_resolution_clock::now() < stepTimer);

        // update step interval
        if(stepCounter < rampUpStep)
        {
            stepInterval -= 2*stepInterval / (4*stepCounter+1);
        }
        else if(stepCounter >= rampDownStep)
        {
            stepInterval += 2*stepInterval / (4 * (stepsRequired-stepCounter) + 1);
        }
        else
        {
            stepInterval = maxInterval;
        }

        // update timer
        stepTimer += std::chrono::microseconds((long int)stepInterval);

        #ifdef DEBUG_MSGS 
            printf("%f\n", stepInterval);
        #endif
    }
}
