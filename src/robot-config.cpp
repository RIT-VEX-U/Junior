#include "robot-config.h"

vex::brain Brain;
vex::controller con;

vex::inertial imu(vex::PORT10, vex::turnType::right);
vex::optical conveyor_optical(vex::PORT17);
vex::triport expander(vex::PORT18);

// Analog sensors
CustomEncoder left_enc{Brain.ThreeWirePort.C, 2048};
CustomEncoder right_enc{Brain.ThreeWirePort.E, 2048};
CustomEncoder front_enc{Brain.ThreeWirePort.G, 2048};

tracking_wheel_cfg_t left_enc_cfg{-0.0625, 3.625, M_PI, 1.0625};
tracking_wheel_cfg_t right_enc_cfg{-0.0625, -3.625, 0, 1.0625};
tracking_wheel_cfg_t front_enc_cfg{4.5, -0.375, (M_PI/2), 1.0625};

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
vex::motor conveyor(vex::PORT9, vex::gearSetting::ratio18_1, true);

std::map<std::string, vex::motor &> motor_names{
  {"left front top", left_front_top},   {"left front bottom", left_front_bottom},   {"left back top", left_back_top},   {"left back bottom", left_back_bottom},

  {"right front top", right_front_top},   {"right front bottom", right_front_bottom},   {"right back top", right_back_top},   {"right back bottom", right_back_bottom},

};

const double intake_volts = 12.0;

void intake(double volts) {
    intake_roller.spin(vex::directionType::fwd, volts, vex::volt);
    intake_ramp.spin(vex::directionType::fwd, volts, vex::volt);
}

void intake() {
    intake_roller.spin(vex::directionType::fwd, intake_volts, vex::volt);
    intake_ramp.spin(vex::directionType::fwd, intake_volts, vex::volt);
}

void outtake(double volts) {
    intake_roller.spin(vex::directionType::rev, volts, vex::volt);
    intake_ramp.spin(vex::directionType::rev, volts, vex::volt);
}

void outtake() {
    intake_roller.spin(vex::directionType::rev, intake_volts, vex::volt);
    intake_ramp.spin(vex::directionType::rev, intake_volts, vex::volt);
}

void conveyor_intake(double volts) {
    conveyor.spin(vex::directionType::fwd, volts, vex::volt);

}

void conveyor_intake() {
    conveyor.spin(vex::directionType::fwd, intake_volts, vex::volt);
}

vex::motor_group left_motors{left_front_top, left_front_bottom, left_back_top, left_back_bottom};
vex::motor_group right_motors{right_front_top, right_front_bottom, right_back_top, right_back_bottom};
// Pneumatics
vex::digital_out goal_grabber_sol{Brain.ThreeWirePort.A};
vex::digital_out ring_pusher_sol{Brain.ThreeWirePort.B};

// ================ SUBSYSTEMS ================

PID::pid_config_t drive_pid_cfg{
    .p = 0.045,
    .i = 0.0,
    .d = 0.0003,
    .deadband = 0.5,
    .on_target_time = 0.1,
};

PID drive_pid{drive_pid_cfg};

PID::pid_config_t turn_pid_cfg{
    .p = 0.01625,
    .i = 0.005,
    .d = 0.001,
    //.deadband = 1.5,
    .deadband = 2,
    .on_target_time = 0.1,
};

PID turn_pid{turn_pid_cfg};

PID::pid_config_t drive_correction_pid{};

robot_specs_t robot_cfg{
    .robot_radius = 12.0,
    .odom_wheel_diam = 2.125,
    .odom_gear_ratio = 1.0,
    .dist_between_wheels = 12.5,

    // .drive_correction_cutoff = 0,
    .drive_feedback = &drive_pid,
    .turn_feedback = &turn_pid,
    .correction_pid = drive_correction_pid,
};

OdometryNWheel<3> odom({left_enc, right_enc, front_enc}, {left_enc_cfg, right_enc_cfg, front_enc_cfg}, &imu, true);
// OdometryTank odom(left_enc, right_enc, robot_cfg, &imu);
TankDrive drive_sys(left_motors, right_motors, robot_cfg, &odom);

/**
 * Main robot initialization on startup. Runs before opcontrol and autonomous are started.
 */
void robot_init()
{
    imu.startCalibration();

    //conveyor_optical.setLight(vex::ledState::on);
    //conveyor_optical.setLightPower(100, vex::percent);

    screen::start_screen(
        Brain.Screen,
        {
            new screen::PIDPage(drive_pid, "drive"),
            new screen::StatsPage(motor_names)
        },
        0
    );
}