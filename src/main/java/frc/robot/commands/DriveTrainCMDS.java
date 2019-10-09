package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveTrainCMDS {
  public static final double SPEEDMOD = 0.5; 

  public static class TankDrive extends Command {

    double rightPower, leftPower;

    public TankDrive() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.driveTrainSubsystem);
    }

    protected void initialize() {}

    @Override
    protected void execute() {
      Robot.driveTrainSubsystem.rightPower(-Robot.oi.stick2.getThrottle() * SPEEDMOD);
      Robot.driveTrainSubsystem.leftPower(-Robot.oi.stick2.getY() * SPEEDMOD);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
    protected void end(){
      Robot.driveTrainSubsystem.leftPower(0);
      Robot.driveTrainSubsystem.rightPower(0);
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
      power = (Robot.oi.stick2.getThrottle() + Robot.oi.stick2.getY()*SPEEDMOD/2);
      // Robot.driveTrainSubsystem.keepDriveStraight(power, power, targetAngle);
      Robot.driveTrainSubsystem.dumbDriveStraight(power);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }
}
