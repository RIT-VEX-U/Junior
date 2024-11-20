#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"


const vex::controller::button &intake_button = con.ButtonL1;
const vex::controller::button &outtake_button = con.ButtonL2;
const vex::controller::button &goal_grabber = con.ButtonUp;
const vex::controller::button &ring_doinker = con.ButtonLeft;

/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{
    // ================ INIT ================
    while (imu.isCalibrating()) {
        vexDelay(1);
    }
    // ================ PERIODIC ================

    intake_button.pressed([]() {
        intake_roller.spin(vex::directionType::fwd, 10, vex::volt);
        intake_ramp.spin(vex::directionType::fwd, 10, vex::volt);
    });

    outtake_button.pressed([]() {
        intake_roller.spin(vex::directionType::rev, 10, vex::volt);
        intake_ramp.spin(vex::directionType::rev, 10, vex::volt);
    });

    goal_grabber.pressed([]() {
        goal_grabber_sol.set(!goal_grabber_sol);
    });

    ring_doinker.pressed([]() {
        ring_pusher_sol.set(!ring_pusher_sol);
    });

    while (true) {
        if(!intake_button.pressing() && !outtake_button.pressing()) {
            intake_roller.stop();
            intake_ramp.stop();
        }

        double straight = (double)con.Axis3.position() / 100;
        double turn = (double)con.Axis1.position() / 100;

        drive_sys.drive_arcade(straight, turn * 0.75, 1, TankDrive::BrakeType::None);

        vexDelay(10);
    }
}