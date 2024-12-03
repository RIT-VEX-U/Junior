#include "competition/opcontrol.h"
#include "vex.h"
#include "robot-config.h"

const vex::controller::button &intake_button = con.ButtonL1;
const vex::controller::button &outtake_button = con.ButtonL2;
const vex::controller::button &goal_grabber = con.ButtonRight;
const vex::controller::button &ring_doinker = con.ButtonY;
const vex::controller::button &conveyor_button = con.ButtonR2;
const vex::controller::button &rev_conveyor_button = con.ButtonR1;

void testing();
/**
 * Main entrypoint for the driver control period
*/
void opcontrol()
{
    testing();
    // ================ INIT ================
    while (imu.isCalibrating()) {
        vexDelay(1);
    }
    // ================ PERIODIC ================

    intake_button.pressed([]() {
        intake();
    });

    outtake_button.pressed([]() {
        outtake();
    });

    conveyor_button.pressed([]() {
        conveyor.spin(vex::directionType::fwd, 12, vex::volt);
    });

    rev_conveyor_button.pressed([]() {
        conveyor.spin(vex::directionType::rev, 12, vex::volt);
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

        if(!conveyor_button.pressing() && !rev_conveyor_button.pressing()) {
            conveyor.stop();
        }

        double straight = (double)con.Axis3.position() / 100;
        double turn = (double)con.Axis1.position() / 100;

        drive_sys.drive_arcade(straight, turn * 0.75, 1, TankDrive::BrakeType::None);

        vexDelay(10);
    }
}

void testing() {
    while (imu.isCalibrating()) {
        vexDelay(1);
    }

    class DebugCommand : public AutoCommand {
    public:
        bool run() override {
            drive_sys.stop();
            pose_t pos = odom.get_position();
            printf("ODO X: %.2f, Y: %.2f, R:%.2f, ", pos.x, pos.y, pos.rot);
            while (true) {
                double f = con.Axis3.position() / 200.0;
                double s = con.Axis1.position() / 200.0;
                drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
                pose_t pos = odom.get_position();
                printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
                // printf(", Left encoder: %.2f", left_enc.position(vex::rotationUnits::deg));
                // printf(", Right encoder: %.2f", right_enc.position(vex::rotationUnits::deg));
                // printf(", Back encoder: %.2f\n", rear_enc.position(vex::rotationUnits::deg));
                vexDelay(100);
            }
            return false;
        }
    };

    con.ButtonA.pressed([]() {
        CommandController cc{
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            drive_sys.DriveForwardCmd(48.0, vex::fwd, 0.6)->withTimeout(3),
            // drive_sys.TurnToHeadingCmd(180, 0.6)->withTimeout(5),
            new DebugCommand(),
        };
        cc.run();
    });

    con.ButtonB.pressed([]() {
        CommandController cc{
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            drive_sys.DriveToPointCmd({96, 0}, vex::fwd, 0.6)->withTimeout(3),
            drive_sys.TurnToHeadingCmd(90, 0.6)->withTimeout(1),
            drive_sys.DriveToPointCmd({48, 96}, vex::fwd, 0.6)->withTimeout(3),
        };
        cc.run();
    });

    while (true) {
        if (conveyor_optical.isNearObject()) {
           con.Screen.print(conveyor_optical.color());
        }
        vexDelay(1);
    }
}