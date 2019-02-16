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
  
  public static class Eject extends Command{

    public Eject(){
      // Use requires() here to declare subsystem dependencies
      //requires(Robot.intakeSubsystem);
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

  public static class StopCMD extends Command {
    public StopCMD() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected void initialize() {}

    @Override
    protected void execute() {
    
        Robot.intakeSubsystem.stop();

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
    protected void execute() {
      Robot.intakeSubsystem.goUp();
    }

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
    protected void execute() {
      Robot.intakeSubsystem.goDown();
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }
public static class AlmostDownCMD extends Command{

  public AlmostDownCMD(){
    requires(Robot.intakeSubsystem);
  }

  protected void initialize(){
    Robot.intakeSubsystem.goAlmostDown();
  }
  protected void execute() {
    Robot.intakeSubsystem.goAlmostDown();
  }
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
        Robot.intakeSubsystem.stow();
      }

    @Override
    protected void execute() {
      Robot.intakeSubsystem.stow();
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }
  public static class ManualControlCMD extends Command{
    public ManualControlCMD() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected void initialize() {
        
      }

    @Override
    protected void execute() {
      Robot.intakeSubsystem.rightIntakeMotor.set(Robot.oi.stick2.getY() * .75);
    }

    @Override
    protected void interrupted() {
      Robot.intakeSubsystem.stop();
      super.interrupted();
    }

    @Override
    protected boolean isFinished() {
      return false;
    }
  }
  
}
