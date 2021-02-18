package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveTrainCMDS {
  public static class TankDrive extends CommandBase {

    double rightPower, leftPower;

    public TankDrive() {
      // Use requires() here to declare subsystem dependencies
      addRequirements(Robot.driveTrainSubsystem);
    }

    @Override
    public void execute() {
      Robot.driveTrainSubsystem.leftPower(Robot.oi.stick.getX());
      Robot.driveTrainSubsystem.rightPower(Robot.oi.stick.getY());
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

}