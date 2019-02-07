package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveTrainCMDS {
  public static class StraightDrive extends Command{
    double power;
    public StraightDrive(){
      requires(Robot.driveTrain);
    }

    protected void initialize(){}

    protected void execute(){
      Robot.driveTrain.leftPower(Robot.oi.stick.getThrottle());
      Robot.driveTrain.rightPower(Robot.oi.stick.getThrottle());
    }

    protected boolean isFinished(){
      return false;
    }
  }
  public static class TankDrive extends Command {

    double rightPower, leftPower;

    public TankDrive() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.driveTrain);
    }

    protected void initialize() {}

    @Override
    protected void execute() {
      Robot.driveTrain.leftPower(Robot.oi.stick.getThrottle());
      Robot.driveTrain.rightPower(Robot.oi.stick.getY());
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }
}
