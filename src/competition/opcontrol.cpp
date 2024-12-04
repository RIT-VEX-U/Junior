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
        intake();
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
        if(!intake_button.pressing() && !outtake_button.pressing() && !conveyor_button.pressing()) {
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
            printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
            printf("ENC LEFT POS: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n", left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg), front_enc.position(vex::rotationUnits::deg));
            while (true) {
                double f = con.Axis3.position() / 200.0;
                double s = con.Axis1.position() / 200.0;
                // double left_enc_start_pos = left_enc.position(vex::rotationUnits::rev);
                drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
                pose_t pos = odom.get_position();
                printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
                // printf("ENC LEFT REV: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n", left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg), front_enc.position(vex::rotationUnits::deg));
                // if (left_enc.position(vex::rotationUnits::rev) >= 1.0) {
                //     break;
                // }
                vexDelay(100);
            }
            return false;
        }
    };

    con.ButtonA.pressed([]() {
        CommandController cc{
            odom.SetPositionCmd({.x=0,.y=0,.rot=0}),
            // drive_sys.DriveToPointCmd({.x=72.0, .y=0}, vex::fwd, 0.6),
            // drive_sys.PurePursuitCmd(PurePursuit::Path({
            //     {.x=48.0, .y=0},
            //     {.x=72.0, .y=0},
            // }, 2), vex::directionType::fwd, 0.6),
            // new RepeatUntil({
            //     drive_sys.DriveForwardCmd(24.0, vex::fwd, 0.6)->withTimeout(12),
            //     new DelayCommand(500),
            // }, new TimesTestedCondition(4)),
            drive_sys.DriveForwardCmd(96.0, vex::fwd, 0.8)->withTimeout(3),
            drive_sys.TurnToHeadingCmd(90.0, 0.7)->withTimeout(3),
            drive_sys.DriveForwardCmd(96.0, vex::fwd, 0.8)->withTimeout(3),
            drive_sys.TurnToHeadingCmd(180.0, 0.7)->withTimeout(3),
            drive_sys.DriveForwardCmd(96.0, vex::fwd, 0.8)->withTimeout(3),
            drive_sys.TurnToHeadingCmd(270.0, 0.7)->withTimeout(3),
            drive_sys.DriveForwardCmd(96.0, vex::fwd, 0.8)->withTimeout(3),
            drive_sys.TurnToHeadingCmd(0.0, 0.7)->withTimeout(3),

            // drive_sys.TurnToHeadingCmd(90, 0.6)->withTimeout(5),
            // new DelayCommand(500),
            // drive_sys.TurnToHeadingCmd(180, 0.6)->withTimeout(5),
            // new DelayCommand(500),
            // drive_sys.TurnToHeadingCmd(270, 0.6)->withTimeout(5),
            // new DelayCommand(500),
            // drive_sys.TurnToHeadingCmd(0, 0.6)->withTimeout(5),
            // new DelayCommand(500),
            // drive_sys.TurnToHeadingCmd(180, 0.7)->withTimeout(5),
            new DebugCommand(),
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