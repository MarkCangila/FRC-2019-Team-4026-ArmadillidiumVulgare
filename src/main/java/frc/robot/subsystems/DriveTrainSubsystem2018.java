package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Portmap;
import frc.robot.commands.DriveTrainCMDS;

public class DriveTrainSubsystem2018 extends Subsystem {

  final Talon rightDriveMotor;
  final Talon leftDriveMotor;
  static final double MAXPOWERCHANGE = 0.0;

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveTrainCMDS.TankDrive());
  }

  public DriveTrainSubsystem2018() {
    leftDriveMotor = new Talon(Portmap.LEFTDRIVETALON);
    rightDriveMotor = new Talon(Portmap.RIGHTDRIVETALON);
    rightDriveMotor.setInverted(true);
  }

  public void leftPower(double requestedPower) {
    double currentPower = leftDriveMotor.get();
    double newPower;
    if (requestedPower < currentPower) {
      newPower = Math.max(requestedPower, currentPower - MAXPOWERCHANGE);
    } else if (requestedPower > currentPower) {
      newPower = Math.min(requestedPower, currentPower + MAXPOWERCHANGE);
    } else {
      newPower = requestedPower;
    }

    leftDriveMotor.set(newPower);
  }

  public void rightPower(double requestedPower) {
    double currentPower = -rightDriveMotor.get();
    double newPower;
    // Ramp up instead of instantly setting power
    if (requestedPower < currentPower) {
      newPower = Math.max(requestedPower, currentPower - MAXPOWERCHANGE);
    } else if (requestedPower > currentPower) {
      newPower = Math.min(requestedPower, currentPower + MAXPOWERCHANGE);
    } else {
      newPower = requestedPower;
    }
    // System.out.println(String.format("Right:requestedPower: %.2f, resultingPower: %.2f,
    // currentPower: %.2f", requestedPower, newPower, currentPower));
    rightDriveMotor.set(newPower);
  }

  // Function to stop instantly, used for auto commands
  public void stop() {
    rightDriveMotor.set(0);
    leftDriveMotor.set(0);
  }
}
