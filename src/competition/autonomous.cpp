#include "competition/autonomous.h"
#include "robot-config.h"

/**
 * Main entrypoint for the autonomous period
 */
void autonomous()
{
	while (imu.isCalibrating()) {
		vexDelay(1);
	}
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

void skills() {
	CommandController cc {
		odom.SetPositionCmd({.x = 12, .y = 72, .rot = 0}),
		intake_command(),
		drive_sys.PurePursuitCmd(PurePursuit::Path({
			{30, 72},
			{58, 30},
		}, 2), vex::forward),
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