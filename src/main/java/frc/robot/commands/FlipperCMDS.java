package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class FlipperCMDS {
  public static class ManualFlip extends Command {
    public ManualFlip() {
      requires(Robot.flipperSubsystem);
    }

    @Override
    protected void execute() {
      Robot.flipperSubsystem.leftFlipper.set(-(Robot.oi.stick2.getThrottle() * 0.5));
      Robot.flipperSubsystem.rightFlipper.set(Robot.oi.stick2.getThrottle() * 0.5);
      Robot.flipperSubsystem.printTelemetry();
    }

    @Override
    protected boolean isFinished() {
      return false;
    }

    @Override
    protected void end() {
      Robot.flipperSubsystem.leftFlipper.set(0);
      Robot.flipperSubsystem.rightFlipper.set(0);
    }
  }
}
