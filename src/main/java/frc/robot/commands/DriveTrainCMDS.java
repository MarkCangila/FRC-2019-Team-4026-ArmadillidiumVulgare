package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Path;
import frc.robot.Portmap;
import frc.robot.Robot;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;

public class DriveTrainCMDS {
  public static class Turn extends Command {
    private float distance;
    private boolean direction;
    // private float goal;
    // private float startingYaw;
    private double startingAngle;
    private final double MAX_TURN_SPEED = 0.4;

    // Dist is in degrees. If direct is true turn right else turn left
    public Turn(float dist, boolean direct) {
      System.out.println("Constructor Called");
      requires(Robot.driveTrainSubsystem);
      // startingYaw = Robot.gyroSubsystem.getRotation();
      startingAngle = Robot.driveTrainSubsystem.getAngle();
      distance = dist;
      direction = direct;
      // This conditional operator adds to the starting yaw when turning right and vice versa when
      // left
      // goal = direction ? startingYaw + distance : startingYaw - distance;
      // If you go past
      // if (goal < -180) {
      // Calculate by how much
      //    float past = Math.abs(goal - (-180)) % 360;
      // And figure out where you should be.
      //    goal = 180 - past;
      // }
      // if (goal > 180) {
      //    float past = Math.abs(goal - 180) % 360;
      //    goal = -(180 - past);
      // }
    }

    @Override
    protected void execute() {
      System.out.println("Executing");
      if (direction) {
        Robot.driveTrainSubsystem.leftPower(MAX_TURN_SPEED);
        Robot.driveTrainSubsystem.rightPower(-MAX_TURN_SPEED);
      } else {
        Robot.driveTrainSubsystem.leftPower(-MAX_TURN_SPEED);
        Robot.driveTrainSubsystem.rightPower(MAX_TURN_SPEED);
      }
    }

    @Override
    protected boolean isFinished() {
      return (float) (Robot.driveTrainSubsystem.getAngle() - startingAngle) > distance;
    }

    @Override
    protected void end() {
      System.out.println("ending");
      Robot.driveTrainSubsystem.stop();
    }

    @Override
    protected void interrupted() {
      System.out.println("Interrupted");
      end();
    }
  }

  public static class TankDrive extends Command {

    double rightPower, leftPower;

    public TankDrive() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.driveTrainSubsystem);
    }

    @Override
    protected void execute() {
      Robot.driveTrainSubsystem.leftPower(Robot.oi.stick.getThrottle());
      Robot.driveTrainSubsystem.rightPower(Robot.oi.stick.getY());
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }

  public static class DriveStraight extends Command {
    double targetAngle;
    double power;

    public DriveStraight() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.driveTrainSubsystem);
    }

    protected void initialize() {
      targetAngle = Robot.driveTrainSubsystem.getAngle();
      // targetAngle = Robot.driveTrainSubsystem.navx.getAngle();
    }

    @Override
    protected void execute() {
      // Power when driving straight is the averaging of the stick values
      power = (Robot.oi.stick.getThrottle() + Robot.oi.stick.getY()) / 2;
      Robot.driveTrainSubsystem.keepDriveStraight(power, power, targetAngle);
      // Robot.driveTrainSubsystem.dumbDriveStraight(power);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }

  public static class ArcadeDriveCMD extends Command {
    double rightPower, leftPower;

    public ArcadeDriveCMD() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.driveTrainSubsystem);
    }

    protected void initialize() {}

    @Override
    protected void execute() {
      rightPower = Robot.oi.stick.getY() - Robot.oi.stick.getZ() * .75;
      leftPower = Robot.oi.stick.getY() + Robot.oi.stick.getZ() * .75;
      rightPower = Portmap.clipOneToOne(rightPower);
      leftPower = Portmap.clipOneToOne(leftPower);
      Robot.driveTrainSubsystem.leftPower(leftPower);
      Robot.driveTrainSubsystem.rightPower(rightPower);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }

  public static class DriveStraightDist extends Command {

    private double heading, ticks, maxPower, minPower;
    private double averageEncoders;
    private boolean isFinished;

    public DriveStraightDist(
        double distanceInch, double maxPowerVal, double minPowerVal, double headingVal) {
      requires(Robot.driveTrainSubsystem);
      ticks = distanceInch * Robot.driveTrainSubsystem.TICKS_PER_INCH;
      heading = headingVal;
      maxPower = maxPowerVal;
      minPower = minPowerVal;
    }

    @Override
    public void initialize() {
      Robot.driveTrainSubsystem.resetEncoders();
    }

    public void execute() {
      System.out.println("Max power " + maxPower);
      averageEncoders =
          (Robot.driveTrainSubsystem.getEncoderLeft() + Robot.driveTrainSubsystem.getEncoderRight())
              / 2;
      double error = ticks - averageEncoders;
      if (error > 150) {
        Robot.driveTrainSubsystem.keepDriveStraight(-maxPower, -maxPower, heading);
      } else if (error < 150 && error > 15) {
        Robot.driveTrainSubsystem.keepDriveStraight(-minPower, -minPower, heading);
      } else if (error < 15) {
        Robot.driveTrainSubsystem.stop();
        isFinished = true;
      }
      System.out.println("Drive Error" + error);
    }

    @Override
    public boolean isFinished() {
      return isFinished;
    }
  }

  public static class FollowPathCMD extends Command {
    private Path path;
    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    public FollowPathCMD(Path inPath) {
      path = inPath;
      requires(Robot.driveTrainSubsystem);
    }

    @Override
    protected void initialize() {
      leftFollower = new EncoderFollower(path.leftTrajectory);
      rightFollower = new EncoderFollower(path.rightTrajectory);

      leftFollower.configureEncoder(
          Robot.driveTrainSubsystem.leftEncoder.get(), Robot.k_ticksPerRev, Robot.k_wheelDiameter);
      rightFollower.configureEncoder(
          Robot.driveTrainSubsystem.rightEncoder.get(), Robot.k_ticksPerRev, Robot.k_wheelDiameter);
      leftFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Robot.k_maxVelocity, 0);
      rightFollower.configurePIDVA(1.0, 0.0, 0.0, 1 / Robot.k_maxVelocity, 0);
    }

    @Override
    protected void execute() {
      double left_speed = leftFollower.calculate(Robot.driveTrainSubsystem.leftEncoder.get());
      double right_speed = rightFollower.calculate(Robot.driveTrainSubsystem.rightEncoder.get());
      // Maybe make this negative if +180 is to the left
      double heading = Robot.driveTrainSubsystem.getAngle();
      double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
      double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
      double turn = 0.8 * (-1.0 / 80.0) * heading_difference;
      Robot.driveTrainSubsystem.leftPower(-(left_speed + turn));
      Robot.driveTrainSubsystem.rightPower(-(right_speed - turn));
    }

    @Override
    protected boolean isFinished() {
      return (leftFollower.isFinished() && rightFollower.isFinished());
    }

    @Override
    protected void end() {
      Robot.driveTrainSubsystem.stop();
    }

    @Override
    protected void interrupted() {
      end();
    }
  }
}
