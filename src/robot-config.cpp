#include "robot-config.h"

vex::brain Brain;
vex::controller con;

vex::inertial imu(vex::PORT10, vex::turnType::right);
vex::optical conveyor_optical(vex::PORT17);
vex::triport expander(vex::PORT18);

// Analog sensors
CustomEncoder left_enc{expander.A, 2048};
CustomEncoder right_enc{expander.B, 2048};
CustomEncoder rear_enc{expander.C, 2048};

tracking_wheel_cfg_t left_enc_cfg{};
tracking_wheel_cfg_t right_enc_cfg{};
tracking_wheel_cfg_t rear_enc_cfg{};

// ================ OUTPUTS ================
// Motors
vex::motor left_front_top(vex::PORT8, vex::gearSetting::ratio6_1, true);
vex::motor left_front_bottom(vex::PORT6, vex::gearSetting::ratio6_1, true);
vex::motor left_back_top(vex::PORT7, vex::gearSetting::ratio6_1, false);
vex::motor left_back_bottom(vex::PORT5, vex::gearSetting::ratio6_1, true);

vex::motor right_front_top(vex::PORT3, vex::gearSetting::ratio6_1, false);
vex::motor right_front_bottom(vex::PORT2, vex::gearSetting::ratio6_1, false);
vex::motor right_back_top(vex::PORT1, vex::gearSetting::ratio6_1, true);
vex::motor right_back_bottom(vex::PORT4, vex::gearSetting::ratio6_1, false);

vex::motor intake_roller(vex::PORT19, vex::gearSetting::ratio6_1, false);
vex::motor intake_ramp(vex::PORT20, vex::gearSetting::ratio6_1, false);
vex::motor conveyor(vex::PORT9, vex::gearSetting::ratio6_1, false);

vex::motor_group left_motors{left_front_top, left_front_bottom, left_back_top, left_back_bottom};
vex::motor_group right_motors{right_front_top, right_front_bottom, right_back_top, right_back_bottom};
// Pneumatics
vex::digital_out goal_grabber_sol{Brain.ThreeWirePort.A};
vex::digital_out ring_pusher_sol{Brain.ThreeWirePort.B};

// ================ SUBSYSTEMS ================

PID::pid_config_t drive_pid_cfg{
    .p = 0,
    .i = 0,
    .d = 0,
    .deadband = 0,
    .on_target_time = 0,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t turn_pid_cfg{
    .p = 0,
    .i = 0,
    .d = 0,
    .deadband = 0,
    .on_target_time = 0,
};

PID turn_pid{turn_pid_cfg};

PID::pid_config_t drive_correction_pid{};

robot_specs_t robot_cfg{
    .robot_radius = 12.0,
    .odom_wheel_diam = 2.0,
    .odom_gear_ratio = 1.0,
    .dist_between_wheels = 11.0,

    // .drive_correction_cutoff = 0,
    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
    .correction_pid = drive_correction_pid,
};

OdometryNWheel<3> odom({left_enc, right_enc, rear_enc}, {left_enc_cfg, right_enc_cfg, rear_enc_cfg}, &imu, true);
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    imu.startCalibration();
}