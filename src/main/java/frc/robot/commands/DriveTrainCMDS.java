package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Portmap;
import frc.robot.Robot;

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
    }

    @Override
    protected void execute() {
      System.out.println("Executing");
      if (direction) {
        Robot.driveTrainSubsystem.leftPower(MAX_TURN_SPEED * .15);
        Robot.driveTrainSubsystem.rightPower(-MAX_TURN_SPEED * .15);
      } else {
        Robot.driveTrainSubsystem.leftPower(-MAX_TURN_SPEED * .15);
        Robot.driveTrainSubsystem.rightPower(MAX_TURN_SPEED* .15);
      }
    }

    @Override
    protected boolean isFinished() {
      // TODO: Confirm that getAngle just counts up. If it doesn't it will be a lot harder and goal
      // will be required.
      // Read the documentation, I have confirmed -Walden
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

    protected void initialize() {}

    @Override
    protected void execute() {
      Robot.driveTrainSubsystem.leftPower(Robot.oi.stick.getThrottle() * .15);
      Robot.driveTrainSubsystem.rightPower(Robot.oi.stick.getY() * .15);
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
      power = (Robot.oi.stick.getThrottle() + Robot.oi.stick.getY()) / 2 * .15;
      // Robot.driveTrainSubsystem.keepDriveStraight(power, power, targetAngle);
      Robot.driveTrainSubsystem.dumbDriveStraight(power);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }
}
