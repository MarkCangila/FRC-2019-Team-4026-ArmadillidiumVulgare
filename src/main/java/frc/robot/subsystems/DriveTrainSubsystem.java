package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Portmap;
import frc.robot.commands.DriveTrainCMDS;

public class DriveTrainSubsystem extends Subsystem {

  final WPI_TalonSRX rightDriveMotorTalon;
  final WPI_TalonSRX leftDriveMotorTalon;
  final VictorSPX rightDriveMotorVictor;
  final VictorSPX leftDriveMotorVictor;
  static final double MAXPOWERCHANGE = .075;

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveTrainCMDS.TankDrive());
  }

  public DriveTrainSubsystem() {
    //Asign Motors
    rightDriveMotorTalon = new WPI_TalonSRX(Portmap.RIGHTDRIVE);
    leftDriveMotorTalon = new WPI_TalonSRX(Portmap.LEFTDRIVE);
    rightDriveMotorVictor = new VictorSPX(Portmap.RIGHTDRIVE);
    leftDriveMotorVictor = new VictorSPX(Portmap.LEFTDRIVE);
    //Set neutral modes
    rightDriveMotorTalon.setNeutralMode(NeutralMode.Brake);
    leftDriveMotorTalon.setNeutralMode(NeutralMode.Brake);
    rightDriveMotorVictor.setNeutralMode(NeutralMode.Brake);
    leftDriveMotorVictor.setNeutralMode(NeutralMode.Brake);
    //Invert motors
    rightDriveMotorTalon.setInverted(true);
    rightDriveMotorVictor.setInverted(true);
    //have victors follow
    rightDriveMotorVictor.follow(rightDriveMotorTalon);
    leftDriveMotorVictor.follow(leftDriveMotorTalon);
  }

  public void leftPower(double requestedPower) {
    double currentPower = leftDriveMotorTalon.get();
    double newPower;
    if (requestedPower < currentPower) {
      newPower = Math.max(requestedPower, currentPower - MAXPOWERCHANGE);
    } else if (requestedPower > currentPower) {
      newPower = Math.min(requestedPower, currentPower + MAXPOWERCHANGE);
    } else {
      newPower = requestedPower;
    }
    System.out.println(
        String.format(
            "Left:requestedPower: %.2f, resultingPower: %.2f, currentPower: %.2f",
            requestedPower, newPower, currentPower));
    leftDriveMotorTalon.set(newPower);
  }

  public void rightPower(double requestedPower) {
    double currentPower = rightDriveMotorTalon.get();
    double newPower;
    if (requestedPower < currentPower) {
      newPower = Math.max(requestedPower, currentPower - MAXPOWERCHANGE);
    } else if (requestedPower > currentPower) {
      newPower = Math.min(requestedPower, currentPower + MAXPOWERCHANGE);
    } else {
      newPower = requestedPower;
    }
    System.out.println(
        String.format(
            "Right:requestedPower: %.2f, resultingPower: %.2f, currentPower: %.2f",
            requestedPower, newPower, currentPower));
    rightDriveMotorTalon.set(newPower);
  }

  // Function to stop instantly
  public void stop() {
    rightDriveMotorTalon.set(0);
    leftDriveMotorTalon.set(0);
  }
}
