package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

// import frc.robot.Robot;

public class TeleOpCMD extends Command {
  private Command TankDriveCMD = null;

  protected Command getTankDriveCMD() {
    if (TankDriveCMD == null) {
      TankDriveCMD = new DriveTrainCMDS.TankDrive();
    }
    return TankDriveCMD;
  }

  @Override
  protected void execute() {
    System.out.println("teleop1");
    // double left = -Robot.oi.stick.getY();
    // double right = -Robot.oi.stick.getThrottle();
    Command driveTrainCMD = getTankDriveCMD();
    driveTrainCMD.start();
    System.out.println("teleop");
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    TankDriveCMD.close();
  }
}
