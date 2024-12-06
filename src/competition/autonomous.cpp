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
		odom.SetPositionCmd({.x = 12, .y = 72, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				pose_t pos = odom.get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
			}
			return true;
		})),

		intake_command(),
		drive_sys.DriveToPointCmd({30, 72}, vex::fwd, 0.6),
		new DebugCommand(),
		// drive_sys.PurePursuitCmd(PurePursuit::Path({
		// 	{30, 72},
		// 	{58, 30},
		// }, 2), vex::forward, 0.6),
		stop_intake(),
		drive_sys.TurnToPointCmd(48, 24, vex::reverse),
		drive_sys.DriveToPointCmd({48, 24}, vex::reverse),
		goal_grabber_command(true),
		intake_command(),
		drive_sys.PurePursuitCmd(PurePursuit::Path({
			{74, 18},
			{72, 12},
		}, 2), vex::forward),
		drive_sys.PurePursuitCmd(PurePursuit::Path({
			{76, 20},
		}, 2), vex::reverse),
		drive_sys.PurePursuitCmd(PurePursuit::Path({
			{24, 24},
			{0, 0},
		}, 2), vex::forward),
		drive_sys.TurnDegreesCmd(180),
		goal_grabber_command(false),
		drive_sys.PurePursuitCmd(PurePursuit::Path({
			{96, 24},
			{120, 24},
		}, 2), vex::forward),
		stop_intake(),
		drive_sys.TurnToPointCmd(96, 48, vex::reverse),
		drive_sys.DriveToPointCmd({96, 48}, vex::reverse),
		goal_grabber_command(true),
		intake_command(),
		drive_sys.PurePursuitCmd(PurePursuit::Path({
			{96, 48},
			{90, 50},
		}, 2), vex::reverse),
		drive_sys.PurePursuitCmd(PurePursuit::Path({
			{120, 48},
			{134, 36},
			{144, 0},
		}, 2), vex::forward),
		drive_sys.TurnDegreesCmd(180),
		goal_grabber_command(false),
	};
	cc.run();
}