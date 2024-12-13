#include "competition/autonomous.h"
#include "robot-config.h"

/**
 * Main entrypoint for the autonomous period
 */

void skills();

void autonomous() {
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
class ConveyorStalled : public Condition {
  bool test() override { return conveyor.current() > 1.5; }
};

AutoCommand *conveyor_intake_command(double amt = 12.0) {
  return new FunctionCommand([=]() {
    conveyor_intake(amt);
    while (conveyor.current() > 1.5) {
      printf("stalls");
      conveyor_intake(amt * -1);
    }
    // waitUntil(conveyor.voltage, => 10);
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
    printf(
      "ENC LEFT POS: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n", left_enc.position(vex::rotationUnits::deg),
      right_enc.position(vex::rotationUnits::deg), front_enc.position(vex::rotationUnits::deg)
    );
    while (true) {
      double f = con.Axis3.position() / 200.0;
      double s = con.Axis1.position() / 200.0;
      // double left_enc_start_pos = left_enc.position(vex::rotationUnits::rev);
      drive_sys.drive_arcade(f, s, 1, TankDrive::BrakeType::None);
      pose_t pos = odom.get_position();
      printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
      // printf("ENC LEFT REV: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n",
      // left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg),
      // front_enc.position(vex::rotationUnits::deg)); if (left_enc.position(vex::rotationUnits::rev) >= 1.0) {
      //     break;
      // }
      vexDelay(100);
    }
    return false;
  }
};

void skills() {
  CommandController cc{
    odom.SetPositionCmd({.x = 11.75, .y = 96, .rot = 0}),

    new Async(new FunctionCommand([]() {
      while (true) {
        pose_t pos = odom.get_position();
        printf("ODO X: %.2f, Y: %.2f, R:%.2f\n", pos.x, pos.y, pos.rot);
        // printf("ENC LEFT REV: %.2f, ENC RIGHT POS: %.2f, ENC BACK POS: %.2f\n",
        // left_enc.position(vex::rotationUnits::deg), right_enc.position(vex::rotationUnits::deg),
        // front_enc.position(vex::rotationUnits::deg));
      }
      return true;
    })),
    // drive_sys.DriveForwardCmd(48, vex::forward, 0.6, 0)
    // First Ring
    // new DebugCommand(),
    conveyor_intake_command(-12),
    intake_command(),
    drive_sys.DriveToPointCmd({48, 96}, vex::forward, .6)->withTimeout(2),
    conveyor_stop_command(),

    // First Stake
    drive_sys.TurnToHeadingCmd(-90, .6)->withTimeout(1.5),
    drive_sys.DriveToPointCmd({48, 120}, vex::reverse, .6)->withTimeout(1.5),
    conveyor_intake_command()->withCancelCondition(new ConveyorStalled),
    goal_grabber_command(true),

    // Second Ring
    drive_sys.TurnToPointCmd(72, 120, vex::directionType::fwd, .6)->withTimeout(.7),
    drive_sys.DriveToPointCmd({72, 120}, vex::forward, .8)->withTimeout(.5),

    // Third Ring
    drive_sys.TurnToHeadingCmd(90, .6)->withTimeout(2),
    // drive_sys.TurnToPointCmd(72, 128, vex::directionType::fwd, .6) -> withTimeout(.7),
    drive_sys.DriveToPointCmd({72, 128}, vex::forward, .6)->withTimeout(1),

    // Fourth Ring
    drive_sys.TurnToPointCmd(24, 120, vex::directionType::fwd, .6)->withTimeout(1),
    drive_sys.DriveToPointCmd({24, 120}, vex::forward, .8)->withTimeout(1.5),

    // Fifth Ring
    drive_sys.TurnToPointCmd(4, 144, vex::directionType::fwd, .6)->withTimeout(1),
    drive_sys.DriveToPointCmd({6, 137}, vex::forward, .8)->withTimeout(1.05),
    // new DebugCommand(),
    odom.SetPositionCmd({.x = 9, .y = 133.5, .rot = 3 * M_PI / 4}),

    // Deposit First Stake
    drive_sys.DriveForwardCmd(18, vex::directionType::rev, .6)->withTimeout(.7),
    new DelayCommand(500),
    // drive_sys.TurnToPointCmd(96, 120, vex::directionType::fwd, .6) -> withTimeout(3),
    drive_sys.TurnToHeadingCmd(315, .5)->withTimeout(1),
    // drive_sys.DriveForwardCmd(15, vex::directionType::rev, .8) -> withTimeout(.3),
    new DelayCommand(500),
    drive_sys.DriveTankCmd(-.5, -.5)->withTimeout(1),
    // drive_sys.DriveForwardCmd(15, vex::directionType::rev, .6) -> withTimeout(.7),
    conveyor_stop_command(),

    // drop goal/move away
    goal_grabber_command(false),

    // new DebugCommand(),

    new DelayCommand(1500),
    // new DebugCommand(),

    odom.SetPositionCmd({.x = 13, .y = 135, .rot = 7 * M_PI / 4}),

    // reset position with two wall aligns
    drive_sys.DriveToPointCmd({26, 122}, vex::forward, .8)->withTimeout(1),
    drive_sys.TurnToHeadingCmd(0, 0.6)->withTimeout(1),
    drive_sys.DriveForwardCmd(28, vex::reverse, 0.4)->withTimeout(2),
    odom.SetPositionCmd({.x = 8, .y = odom.get_position().y, 0}),
    drive_sys.DriveForwardCmd(24, vex::forward, 0.4)->withTimeout(2),
    drive_sys.TurnToHeadingCmd(-90, 0.4)->withTimeout(2),
    drive_sys.DriveForwardCmd(28, vex::reverse, 0.4)->withTimeout(2),
    odom.SetPositionCmd({.x = odom.get_position().x, .y = (144 - 8), rot = ((2 * M_PI) / 2)}),

    drive_sys.DriveToPointCmd({odom.get_position().x, 120}, vex::forward, 0.6)->withTimeout(1),

    // new DebugCommand(),

    // first ring second goal
    drive_sys.TurnToPointCmd(96, 120, vex::forward, .6)->withTimeout(1),
    // new DebugCommand(),
    //  drive_sys.DriveForwardCmd(24, vex::directionType::fwd, .6) -> withTimeout(1.5),
    //  drive_sys.TurnToPointCmd(96, 120, vex::forward, .6) -> withTimeout(1),
    //  new DelayCommand(2000),
    //  drive_sys.DriveForwardCmd(24, vex::directionType::fwd, .6) -> withTimeout(1.5),
    //  drive_sys.TurnToPointCmd(96, 120, vex::forward, .6) -> withTimeout(1),
    //  new DelayCommand(2000),
    //\\ drive_sys.DriveToPointCmd({96, 120}, vex::forward, .6) -> withTimeout(2.5),
    // new DebugCommand(),
    drive_sys
      .PurePursuitCmd(
        PurePursuit::Path(
          {
            {.x = 48, .y = 120},
            {.x = 72, .y = 120},
            {.x = 94, .y = 120},

          },
          2
        ),
        vex::fwd, 0.4
      )
      ->withTimeout(3.5),
    // new DebugCommand(),
    // grab second goal
    drive_sys.TurnToHeadingCmd(90, .3)->withTimeout(2),
    // drive_sys.TurnToHeadingCmd(90, .4) -> withTimeout(1),
    drive_sys.DriveToPointCmd({96, 76}, vex::directionType::rev, .3)->withTimeout(1.5),
    // new DebugCommand(),
    new DelayCommand(1000),
    goal_grabber_command(true),

    // new DebugCommand(),
    conveyor_intake_command()->withCancelCondition(new ConveyorStalled()),

    //@Todo IF MESSED UP CHANGE NEXT 2 LINES!!!!!!!!!!

    // drive_sys.DriveForwardCmd(6, vex::directionType::rev, .4) -> withTimeout(1),
    // odom.SetPositionCmd({.x = 96, .y = 96, .rot = 90}),
    // new DebugCommand(),

    // second ring#2
    drive_sys.TurnToHeadingCmd(0, .6)->withTimeout(1.5),
    drive_sys.DriveToPointCmd({120, 96}, vex::forward, 0.6)->withTimeout(1),

    // third ring#2
    drive_sys.TurnToHeadingCmd(90, .6)->withTimeout(2),
    drive_sys.DriveToPointCmd({120, 120}, vex::forward, 0.6)->withTimeout(1),

    // corner
    drive_sys.TurnToHeadingCmd(45, .6)->withTimeout(1),

    drive_sys.DriveTankCmd(.5, .5)->withTimeout(1.5),
    // odom.SetPositionCmd({.x = 135, .y = 133.5, .rot = M_PI/4}),
    // drive_sys.DriveForwardCmd(18,vex::directionType::fwd,.6)-> withTimeout(1),
    conveyor_stop_command(),

    // spit out blue
    drive_sys.DriveForwardCmd(12, vex::directionType::rev, .6)->withTimeout(1),
    drive_sys.TurnToHeadingCmd(135, .6)->withTimeout(1),
    outtake_command(12),
    drive_sys.DriveForwardCmd(10, vex::directionType::fwd, 1)->withTimeout(.3),
    drive_sys.DriveForwardCmd(8, vex::directionType::rev, .8)->withTimeout(1),

    // 4th ring
    drive_sys.TurnToPointCmd(144, 144, vex::directionType::fwd, .6)->withTimeout(1),
    intake_command(),
    // drive_sys.DriveToPointCmd({134, 134}, vex::forward, .8) -> withTimeout(1),
    drive_sys.DriveTankCmd(.6, .6)->withTimeout(1),

    // drop goal in corner
    drive_sys.DriveForwardCmd(4, vex::directionType::rev, .6)->withTimeout(1),
    drive_sys.TurnToHeadingCmd(-135, .6)->withTimeout(1),
    goal_grabber_command(false),
    new DelayCommand(1500),
    drive_sys.DriveForwardCmd(8, vex::directionType::fwd, .6)->withTimeout(1),
    new DelayCommand(1500),

    goal_grabber_command(true),
    new DelayCommand(1500),
    drive_sys.DriveForwardCmd(12, vex::directionType::rev, .6)->withTimeout(1),

    new DelayCommand(1500),
    drive_sys.DriveForwardCmd(24, vex::directionType::fwd, .6)->withTimeout(1),

    new DebugCommand(),

    // drive away from goal
    drive_sys.TurnToPointCmd(120, 120, vex::directionType::fwd, .6)->withTimeout(1),
    drive_sys.DriveToPointCmd({120, 120}, vex::forward, .6)->withTimeout(1),
    goal_grabber_command(false),

    // new DebugCommand(),

    // grab third goal
    drive_sys.TurnToHeadingCmd(90, .6)->withTimeout(1),
    drive_sys.DriveToPointCmd({120, 72}, vex::reverse, .6)->withTimeout(1.4),
    goal_grabber_command(true),

    // first ring#3
    intake_command(),
    conveyor_intake_command(),
    drive_sys.TurnToHeadingCmd(0, .6)->withTimeout(1.5),
    drive_sys.DriveForwardCmd(10, vex::directionType::fwd, .4)->withTimeout(1),
    drive_sys.DriveForwardCmd(10, vex::reverse, 0.4)->withTimeout(1),

    // gotta go under the ladder
    drive_sys.TurnToPointCmd(96, 96, vex::forward, 0.6)->withTimeout(1),
    drive_sys.DriveToPointCmd({96, 96}, vex::forward, 0.6)->withTimeout(1),
    drive_sys.TurnToHeadingCmd(-135, 0.6)->withTimeout(1),
    // first ring
    drive_sys.DriveForwardCmd(30, vex::forward, 0.6)->withTimeout(1),
    // second ring
    drive_sys.TurnToHeadingCmd(180, 0.4)->withTimeout(1),
    drive_sys.DriveForwardCmd(10, vex::forward, 0.4)->withTimeout(1),
    // third ring
    drive_sys.TurnToHeadingCmd(-90, 0.4)->withTimeout(1),
    drive_sys.DriveForwardCmd(10, vex::forward, 0.4)->withTimeout(1),
    // fourth ring
    drive_sys.TurnToHeadingCmd(0, 0.4)->withTimeout(1),
    drive_sys.DriveForwardCmd(10, vex::forward, 0.4)->withTimeout(1),

    new DelayCommand(700),
    goal_grabber_command(false),
  };

  cc.run();

  while (true) {
    if (new ConveyorStalled) {
      printf("Conveyor Stalled");
      conveyor_intake(-12);
      vexDelay(500);
      conveyor_intake(12);
    }
  }
}