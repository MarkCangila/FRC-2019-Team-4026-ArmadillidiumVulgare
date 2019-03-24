/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/** An example command. You can replace me with your own command. */
public class HatchGrabberCMDS {

  public static class ReleaseHatch extends Command {
    Timer releaseTimer;
    Boolean isFinished;

    public ReleaseHatch() {
      // Use requires() here to declare subsystem dependencies
      requires(Robot.intakeSubsystem);
      releaseTimer = new Timer();
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
      Robot.intakeSubsystem.releaseHatch();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
      Robot.intakeSubsystem.releaseHatch();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
      return false;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {}

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
      end();
    }
  }

  public static class AutoGrabHatch extends Command {

    boolean isFinished = false;
    boolean grabDaHatch = false;

    public AutoGrabHatch() {
      requires(Robot.intakeSubsystem);
    }

    @Override
    protected void initialize() {
      Robot.intakeSubsystem.releaseHatch();
      grabDaHatch = false;
    }

    protected void execute() {
      if (!grabDaHatch) {
        if (Robot.intakeSubsystem.armGrabber()) {
          grabDaHatch = true;
        } else {

        }
      } else {
        Robot.intakeSubsystem.grabHatch();
      }
    }

    protected boolean isFinished() {
      return false;
    }
  }

  public static class ManualGrabCMD extends Command {
    public ManualGrabCMD() {
      requires(Robot.intakeSubsystem);
    }

    protected void initialize() {}

    protected void execute() {
      Robot.intakeSubsystem.grabHatch();
    }

    @Override
    protected boolean isFinished() {
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
      Robot.intakeSubsystem.moveGrabberUp();
    }

    @Override
    protected void execute() {
      Robot.intakeSubsystem.moveGrabberUp();
    }

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
}
