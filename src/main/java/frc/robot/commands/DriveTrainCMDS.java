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
      Robot.driveTrainSubsystem.leftPower(Robot.oi.stick.getThrottle());
      Robot.driveTrainSubsystem.rightPower(Robot.oi.stick.getY());
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }

  public static class DriveStraight extends Command {

    double power;

    public DriveStraight() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.driveTrainSubsystem);
    }

    protected void initialize() {}

    @Override
    protected void execute() {
      power = (Robot.oi.stick.getThrottle() + Robot.oi.stick.getY()) / 2; 
      Robot.driveTrainSubsystem.leftPower(power);
      Robot.driveTrainSubsystem.rightPower(power);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }
}