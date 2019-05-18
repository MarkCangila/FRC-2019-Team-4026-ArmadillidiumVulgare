package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveTrainCMDS {

  public static class TankDrive extends Command {

    double rightPower, leftPower;

    public TankDrive() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.driveTrainSubsystem);
    }

    protected void initialize() {}

    @Override
    protected void execute() {
      Robot.driveTrainSubsystem.rightPower(-Robot.oi.stick.getThrottle() * .15);
      Robot.driveTrainSubsystem.leftPower(-Robot.oi.stick.getY() * .15);
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
