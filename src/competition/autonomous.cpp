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
class ConveyorStalled : public Condition{
	bool test() override{
		return conveyor.current() > 1.5;
	}
};

AutoCommand *conveyor_intake_command(double amt = 12.0) {
	return new FunctionCommand([=]() {
		conveyor_intake(amt);
		while(conveyor.current() > 1.5){
			printf("stalls");
			conveyor_intake(amt * -1);
		}
	//waitUntil(conveyor.voltage, => 10);
	//	conveyor_intake(-10);
	//	wait(.3,sec);
	//	conveyor_intake(amt);
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
		stop_intake();
		pose_t pos = odom.get_position();
		printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
		printf("ENC LEFT POS: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n", left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg));
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
		odom.SetPositionCmd({.x =11.75, .y = 96, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				pose_t pos = odom.get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
				//printf("ENC LEFT REV: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n", left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg), front_enc.position(vex::rotationUnits::deg));
			}
			return true;
		})),
	// drive_sys.DriveForwardCmd(48, vex::forward, 0.6, 0)
	// First Ring
	// new DebugCommand(),
	conveyor_intake_command(-12),
	intake_command(),
	

	};
	 cc.run();

	 while(true){
		if(new ConveyorStalled){
			printf("Conveyor Stalled");
			conveyor_intake(-12);
			vexDelay(500);
			conveyor_intake(12);
		}
	 }
}