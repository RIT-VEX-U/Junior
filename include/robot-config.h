#pragma once
#include "vex.h"
#include "core.h"

extern vex::brain brain;
extern vex::controller con;

// ================ INPUTS ================
// Digital sensors
extern vex::inertial imu;
extern vex::optical conveyor_optical;

// Analog sensors
extern CustomEncoder left_side;
extern CustomEncoder right_side;

// ================ OUTPUTS ================
// Motors
extern motor left_front;
extern motor left_middle;
extern motor left_back;

extern motor right_front;
extern motor right_middle;
extern motor right_back;

extern motor conveyor;
extern motor intake_roller;
extern motor intake_ramp;

extern motor_group left_motors;
extern motor_group right_motors;
// Pneumatics
extern digital_out goal_grabber;
extern digital_out ring_pusher;

// ================ SUBSYSTEMS ================
extern OdometryTank odom;
extern TankDrive drive_sys;
extern PID drive_pid;

extern robot_specs_t robot_cfg;
// ================ UTILS ================

void robot_init();