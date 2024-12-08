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
		odom.SetPositionCmd({.x = 9, .y = 96, .rot = 0}),

		new Async(new FunctionCommand([]() {
			while(true) {
				pose_t pos = odom.get_position();
            	printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
			}
			return true;
		})),
	// drive_sys.DriveForwardCmd(48, vex::forward, 0.6, 0)
	// First Ring
	conveyor_intake_command(-12),
	intake_command(),
	drive_sys.DriveToPointCmd({48, 96}, vex::forward, .6) -> withTimeout(2),
	conveyor_stop_command(),

	// First Stake
	drive_sys.TurnToHeadingCmd(-90, .6) -> withTimeout(2),
	drive_sys.DriveToPointCmd({48, 120}, vex::reverse, .6) -> withTimeout(2),
	conveyor_intake_command()->withCancelCondition(new ConveyorStalled),
	goal_grabber_command(true),

	// Second Ring
	drive_sys.TurnToPointCmd(72, 120, vex::directionType::fwd, .6) -> withTimeout(.7),
	drive_sys.DriveToPointCmd({72, 120}, vex::forward, .8) -> withTimeout(.5),

	// Third Ring
	drive_sys.TurnToHeadingCmd(90, .6) -> withTimeout(2),
	//drive_sys.TurnToPointCmd(72, 128, vex::directionType::fwd, .6) -> withTimeout(.7),
	drive_sys.DriveToPointCmd({72, 128}, vex::forward, .6) -> withTimeout(1),

	// Fourth Ring
	drive_sys.TurnToPointCmd(24, 120, vex::directionType::fwd, .6) -> withTimeout(1),
	drive_sys.DriveToPointCmd({24, 120}, vex::forward, .8) -> withTimeout(1),

	// Fifth Ring
	drive_sys.TurnToPointCmd(4, 144, vex::directionType::fwd, .6) -> withTimeout(1),
	drive_sys.DriveToPointCmd({6, 137}, vex::forward, .8) -> withTimeout(1),
	//new DebugCommand(),
	odom.SetPositionCmd({.x = 9, .y = 133.5, .rot = 130.5}),
		
	// Deposit First Stake
	drive_sys.DriveForwardCmd(18, vex::directionType::rev, .6) -> withTimeout(.7),
	// drive_sys.TurnToPointCmd(96, 120, vex::directionType::fwd, .6) -> withTimeout(3),
	drive_sys.TurnToHeadingCmd(100, .6) -> withTimeout(.5),
	//drive_sys.DriveForwardCmd(15, vex::directionType::rev, .8) -> withTimeout(.3),
	drive_sys.DriveTankCmd(-.6,-.6) -> withTimeout(.5),
	//drive_sys.DriveForwardCmd(15, vex::directionType::rev, .6) -> withTimeout(.7),
	conveyor_stop_command(),
	
	

	//drop goal/move away
	goal_grabber_command(false),
	
	//new DebugCommand(),
	
	new DelayCommand(1500),
	new DebugCommand(),
	odom.SetPositionCmd({.x= 13, .y=129, .rot = -315}),
	drive_sys.DriveForwardCmd(5, vex::directionType::fwd, .6) -> withTimeout(.5),
	drive_sys.DriveToPointCmd({24, 120}, vex::forward, .8) -> withTimeout(.6),
	
	

	
 	//first ring second goal
 	drive_sys.TurnToPointCmd(96, 120, vex::forward, .6) -> withTimeout(1),
	//new DebugCommand(),
 	drive_sys.DriveToPointCmd({96, 120}, vex::forward, .6) -> withTimeout(2.5),
	new DebugCommand(),

 	//grab second goal
 	drive_sys.TurnToHeadingCmd(90, .6) -> withTimeout(1),
 	drive_sys.DriveToPointCmd({96, 96},vex::directionType::rev,.6)-> withTimeout(1),
 	goal_grabber_command(true),
	new DebugCommand(),
 	conveyor_intake_command()->withCancelCondition(new ConveyorStalled()),
	//new DebugCommand(),

 	//second ring#2
 	drive_sys.TurnToHeadingCmd(0, .6) -> withTimeout(1),
 	drive_sys.DriveForwardCmd(24,vex::directionType::fwd,.6)-> withTimeout(1),
		
 	//third ring#2
 	drive_sys.TurnToHeadingCmd(90, .6) -> withTimeout(1),
 	drive_sys.DriveForwardCmd(18,vex::directionType::fwd,.6)-> withTimeout(1),
		
 	//corner
 	drive_sys.TurnToHeadingCmd(45, .6) -> withTimeout(1),
 	drive_sys.DriveForwardCmd(18,vex::directionType::fwd,.6)-> withTimeout(1),
 	conveyor_stop_command(),

 	//spit out blue
 	drive_sys.DriveForwardCmd(12,vex::directionType::rev,.6)-> withTimeout(1),
 	drive_sys.TurnToHeadingCmd(135, .6) -> withTimeout(1),
 	outtake_command(12),
 	drive_sys.DriveForwardCmd(10,vex::directionType::fwd,1)-> withTimeout(.3),
 	drive_sys.DriveForwardCmd(8,vex::directionType::rev,.8)-> withTimeout(1),

 	//4th ring
 	drive_sys.TurnToPointCmd(144, 144, vex::directionType::fwd, .6) -> withTimeout(1),
 	intake_command(),
 	drive_sys.DriveToPointCmd({134, 134}, vex::forward, .8) -> withTimeout(1),

 	//drop goal in corner
 	drive_sys.DriveForwardCmd(4,vex::directionType::rev,.8)-> withTimeout(1),
 	drive_sys.TurnToHeadingCmd(-135, .6) -> withTimeout(1),
 	goal_grabber_command(false),
 	drive_sys.DriveForwardCmd(4,vex::directionType::fwd,.8)-> withTimeout(1),
 	goal_grabber_command(true),
 	drive_sys.DriveForwardCmd(12,vex::directionType::rev,.6)-> withTimeout(1),


 	//drive away from goal
 	drive_sys.TurnToPointCmd(120, 120, vex::directionType::fwd, .6) -> withTimeout(1),
 	drive_sys.DriveToPointCmd({120, 120}, vex::forward, .8) -> withTimeout(1),
 	goal_grabber_command(false),
		
 	//grab third goal
 	drive_sys.TurnToHeadingCmd(90, .6) -> withTimeout(1),
 	drive_sys.DriveToPointCmd({120, 70}, vex::forward, .8) -> withTimeout(1.4),
 	goal_grabber_command(true),

 	//first ring#3
 	drive_sys.TurnToHeadingCmd(0, .6) -> withTimeout(1.5),
 	intake_command(),
 	conveyor_intake_command(),
 	drive_sys.DriveForwardCmd(10,vex::directionType::fwd,.6)-> withTimeout(1.5),



		
	 	new FunctionCommand([=]() {
	 		vexDelay(700);
	 		return true;
	 	}),
	 	goal_grabber_command(false),
	 	odom.SetPositionCmd({.x = 23, .y = 129, .rot = -45}),
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