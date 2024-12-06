#include "competition/autonomous.h"
#include "robot-config.h"

/**
 * Main entrypoint for the autonomous period
 */

void skills();

void autonomous()
{
	while (imu.isCalibrating()) {
		vexDelay(1);
	}

	skills();
}

AutoCommand *intake_command(double amt = 12.0) {
	return new FunctionCommand([=]() {
		intake(amt);
		return true;
	});
}

AutoCommand *outtake_command(double amt = 12.0) {
	return new FunctionCommand([=]() {
		outtake(amt);
		return true;
	});
}

AutoCommand *stop_intake() {
	return new FunctionCommand([=]() {
		intake(0);
		return true;
	});
}

AutoCommand *conveyor_intake_command(double amt = 12.0) {
	return new FunctionCommand([=]() {
		conveyor_intake(amt);
		return true;
	});
}

AutoCommand *conveyor_stop_command() {
	return new FunctionCommand([=]() {
		conveyor_intake(0);
		return true;
	});
}

AutoCommand *goal_grabber_command(bool value) {
	return new FunctionCommand([=]() {
		goal_grabber_sol.set(value);
		return true;
	});
}

AutoCommand *ring_pusher_command(bool value) {
	return new FunctionCommand([=]() {
		ring_pusher_sol.set(value);
		return true;
	});
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

void skills() {
	CommandController cc {
		odom.SetPositionCmd({.x = 12, .y = 96, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				pose_t pos = odom.get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
			}
			return true;
		})),

		// First Ring
		intake_command(),
		drive_sys.DriveToPointCmd({48, 96}, vex::forward, .6) -> withTimeout(2),

		// First Stake
		drive_sys.TurnToHeadingCmd(-90, .6) -> withTimeout(2),
		drive_sys.DriveToPointCmd({48, 120}, vex::reverse, .6) -> withTimeout(2),
		conveyor_intake_command(),
		goal_grabber_command(true),

		// Second Ring
		drive_sys.TurnToPointCmd(72, 120, vex::directionType::fwd, .6) -> withTimeout(2),
		drive_sys.DriveToPointCmd({72, 120}, vex::forward, .6) -> withTimeout(2),

		// Third Ring
		drive_sys.TurnToHeadingCmd(90, .6) -> withTimeout(2),
		drive_sys.DriveToPointCmd({72, 128}, vex::forward, .6) -> withTimeout(2),

		// Fourth Ring
		drive_sys.TurnToPointCmd(24, 120, vex::directionType::fwd, .6) -> withTimeout(2),
		drive_sys.DriveToPointCmd({24, 120}, vex::forward, .6) -> withTimeout(2),

		// Fifth Ring
		drive_sys.TurnToPointCmd(0, 144, vex::directionType::fwd, .6) -> withTimeout(2),
		drive_sys.DriveToPointCmd({8, 136}, vex::forward, .6) -> withTimeout(2),

		// Deposit First Stake
		drive_sys.DriveForwardCmd(8, vex::directionType::rev, .6) -> withTimeout(2),
		// drive_sys.TurnToPointCmd(96, 120, vex::directionType::fwd, .6) -> withTimeout(3),
		drive_sys.TurnToHeadingCmd(-45, .6) -> withTimeout(2),
		new FunctionCommand([=]() {
			vexDelay(700);
			return true;
		}),
		goal_grabber_command(false),
		odom.SetPositionCmd({.x = 23, .y = 129, .rot = -45}),
	};
	cc.run();
}