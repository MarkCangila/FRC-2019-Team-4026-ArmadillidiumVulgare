package frc.robot.commands;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command. You can replace me with your own command. */
public class ShooterCMDS {
  public static class ToggleServo extends Command{

    @Override
    protected void execute() {
      if (Robot.intakeSubsystem.status == IntakeSubsystem.servoState.CLOSED){
        Robot.intakeSubsystem.ballFeeder.set(0);
        Robot.intakeSubsystem.status = IntakeSubsystem.servoState.OPEN;
      }
      else {
        Robot.intakeSubsystem.ballFeeder.set(90);
        Robot.intakeSubsystem.status = IntakeSubsystem.servoState.CLOSED;
      }
    }
    @Override
    protected boolean isFinished() {
      return false;
    }

  }

  public static class Stop extends Command{
    public Stop(){
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }

    @Override
    protected void execute() {
      Robot.intakeSubsystem.frontIntakeMotor.set(0);
      Robot.intakeSubsystem.backIntakeMotor.set(0);
    }
  }
  public static class ShootSlow extends Command {
    public ShootSlow() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected boolean isFinished() {
      return false;
    }

    @Override
    protected void initialize() {}

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      Robot.intakeSubsystem.frontIntakeMotor.set(-0.25);
      Robot.intakeSubsystem.backIntakeMotor.set(0.35);
    }

  }

  public static class ShootFast extends Command {
    

    public ShootFast() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.intakeSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {}

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      Robot.intakeSubsystem.frontIntakeMotor.set(-0.5);
      Robot.intakeSubsystem.backIntakeMotor.set(.6);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
      Robot.intakeSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
      end();
    }
  }
}
