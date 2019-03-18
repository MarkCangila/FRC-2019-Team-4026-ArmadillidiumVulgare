package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.commands.DriveTrainCMDS;

public class DriveTrainSubsystem2019 extends DriveTrain {
  public final double TICKS_PER_INCH = 13.3333333333333;
  final WPI_TalonSRX rightDriveMotorTalon;
  final WPI_TalonSRX leftDriveMotorTalon;
  final VictorSPX rightDriveMotorVictor;
  final VictorSPX leftDriveMotorVictor;

  Encoder rightEncoder;
  Encoder leftEncoder;

  static final double MAXPOWERCHANGE = .075;

  public AnalogGyro navx;

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveTrainCMDS.TankDrive());
  }

  public DriveTrainSubsystem2019() {
    //  System.out.println();

    navx = new AnalogGyro(Portmap.GYRO);

    // This assumes the robot will start backwards at the beginning of the match.
    // navx.setAngleAdjustment(180);

    rightEncoder = new Encoder(Portmap.RIGHT_ENCODER_1, Portmap.RIGHT_ENCODER_2, false);
    leftEncoder = new Encoder(Portmap.LEFT_ENCODER_1, Portmap.LEFT_ENCODER_2, true);

    rightDriveMotorTalon = new WPI_TalonSRX(Portmap.RIGHTDRIVETALON);
    leftDriveMotorTalon = new WPI_TalonSRX(Portmap.LEFTDRIVETALON);
    rightDriveMotorVictor = new VictorSPX(Portmap.RIGHTDRIVEVICTOR);
    leftDriveMotorVictor = new VictorSPX(Portmap.LEFTDRIVEVICTOR);
    // rightDriveMotorVictor.follow(rightDriveMotorTalon);
    // leftDriveMotorVictor.follow(leftDriveMotorTalon);
    // rightDriveMotorTalon.setInverted(true);
    rightDriveMotorTalon.setNeutralMode(NeutralMode.Brake);
    leftDriveMotorTalon.setNeutralMode(NeutralMode.Brake);
    rightDriveMotorVictor.setNeutralMode(NeutralMode.Brake);
    leftDriveMotorVictor.setNeutralMode(NeutralMode.Brake);
    rightDriveMotorTalon.setInverted(true);
    rightDriveMotorVictor.setInverted(true);
    rightDriveMotorVictor.follow(rightDriveMotorTalon);
    leftDriveMotorVictor.follow(leftDriveMotorTalon);

    navx.calibrate();
    resetEncoders();
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
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
    // System.out.println(
    //  String.format(
    //    "Left:requestedPower: %.2f, resultingPower: %.2f, currentPower: %.2f",
    //  requestedPower, newPower, currentPower));
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
    // System.out.println(
    //  String.format(
    //    "Right:requestedPower: %.2f, resultingPower: %.2f, currentPower: %.2f",
    //  requestedPower, newPower, currentPower));
    rightDriveMotorTalon.set(newPower);
  }

  // Function to stop instantly
  public void stop() {
    rightDriveMotorTalon.set(0);
    leftDriveMotorTalon.set(0);
  }

  private void updateSmartDashboard() {
    Sendable dataForGyro = navx;

    // SmartDashboard.putBoolean("NAVX CONNECTED", navx.isConnected());
    SmartDashboard.putData("Gyro", dataForGyro);
    //SmartDashboard.putNumber("Heading", navx.getAngle());
    //SmartDashboard.putNumber("Right Encoder", rightEncoder.get());
    //SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
  }

  public void dumbDriveStraight(double power) {
    rightPower(power);
    leftPower(power);
  }
  /*
   * Please note that powers must be negative to move foreward, this is a side effect of the controller axies being reversed
   * from what would be accepted.
   */
  public void keepDriveStraight(double leftDriveVel, double rightDriveVel, double targetAngle) {
    System.out.println("Drive straight " + leftDriveVel);
    double error = 0, correctionFactor;
    error = targetAngle - navx.getAngle();
    correctionFactor = (error / 2);

    // todo - best practice - conditions on a separate line should be
    // wrapped in brackets
    if (leftDriveVel > 0.9) leftDriveVel = 0.9;
    else if (leftDriveVel < -0.9) leftDriveVel = -0.9;

    // todo - best practice - conditions on a separate line should be
    // wrapped in brackets
    if (rightDriveVel > 0.9) rightDriveVel = 0.9;
    else if (rightDriveVel < -0.9) rightDriveVel = -0.9;

    if (targetAngle > (navx.getAngle() - 0.5) || targetAngle < (navx.getAngle() + 0.5)) {
      rightPower(((rightDriveVel) - correctionFactor));
      leftPower((leftDriveVel + correctionFactor));
    } else {
      rightPower(rightDriveVel);
      leftPower(leftDriveVel);
    }
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public int getEncoderRight() {
    return rightEncoder.get();
  }

  public int getEncoderLeft() {
    return leftEncoder.get();
  }

  public void resetEncoders() {
    rightEncoder.reset();
    leftEncoder.reset();
  }
}
