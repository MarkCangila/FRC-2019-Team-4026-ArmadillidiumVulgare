/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DoubleSolenoid;


/** An example command. You can replace me with your own command. */
public class HatchGrabberCMDS {
  public static class Intake extends Command {
    long stopTime_NS;

    public Intake() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.intakeSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {}

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      Robot.intakeSubsystem.intake();
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
  public static class Eject extends Command{

    public Eject(){
      // Use requires() here to declare subsystem dependencies
      requires(Robot.intakeSubsystem);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
      
    }
      
    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      Robot.intakeSubsystem.ejector.set(DoubleSolenoid.Value.kForward);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
      Robot.intakeSubsystem.ejector.set(DoubleSolenoid.Value.kReverse);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
      end();
    }
  }

  public static class Outake extends Command {
    public Outake() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    protected void execute() {
      Robot.intakeSubsystem.outtake();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }

  public static class StopCMD extends Command {
    public StopCMD() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected void initialize() {}

    @Override
    protected void execute() {
      if (Robot.intakeSubsystem.rightIntakeMotor.getControlMode() == ControlMode.Position) {
        Robot.intakeSubsystem.goUp();
      } else {
        Robot.intakeSubsystem.stop();
      }

      Robot.intakeSubsystem.ejector.set(DoubleSolenoid.Value.kReverse);

      SmartDashboard.putNumber(
          "Absolute Position",
          Robot.intakeSubsystem.rightIntakeMotor.getSensorCollection().getPulseWidthPosition());
      SmartDashboard.putNumber(
          "Encoder Ticks Relative",
          Robot.intakeSubsystem.rightIntakeMotor.getSelectedSensorPosition(0));
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }

  public static class GoUpCMD extends Command {
    public GoUpCMD() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected void initialize() {
      Robot.intakeSubsystem.goUp();
    }

    @Override
    protected void execute() {}

    @Override
    protected boolean isFinished() {
      return false;
    }
  }

  public static class GoDownCMD extends Command {
    public GoDownCMD() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected void initialize() {
      Robot.intakeSubsystem.goDown();
    }

    @Override
    protected void execute() {}

    @Override
    protected boolean isFinished() {
      return false;
    }
  }

  public static class StowCMD extends Command {
    public StowCMD() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected void initialize() {
      if (Robot.intakeSubsystem.stowed) {
        Robot.intakeSubsystem.unStow();
      } else {
        Robot.intakeSubsystem.stow();
      }
    }

    @Override
    protected void execute() {}

    @Override
    protected boolean isFinished() {
      return false;
    }
  }
}
