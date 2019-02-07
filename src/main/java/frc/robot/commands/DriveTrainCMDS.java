package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
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
      startingAngle = Robot.gyroSubsystem.getAngle();
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
      // TODO: Confirm that getAngle just counts up. If it doesn't it will be a lot harder and goal
      // will be required.
      return (float) (Robot.gyroSubsystem.getAngle() - startingAngle) > distance;
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
