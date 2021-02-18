package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.Robot;
import frc.robot.commands.DriveTrainCMDS;
import frc.robot.Constants;

public class DriveTrainSubsystem2019 extends DriveTrain {
  public final double TICKS_PER_INCH = 13 + (1/3);
  final WPI_TalonSRX rightDriveMotorTalon;
  final WPI_TalonSRX leftDriveMotorTalon;
  final BaseMotorController rightDriveMotorVictor;
  final BaseMotorController leftDriveMotorVictor;
  final DifferentialDrive drive;

  Encoder rightEncoder;
  Encoder leftEncoder;

  static final double MAXPOWERCHANGE = .16 ;

  public AnalogGyro navx;

  public final DifferentialDriveOdometry odometry;

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveTrainCMDS.TankDrive());
  }

  public DriveTrainSubsystem2019(boolean practice) {
  //  System.out.println();

    navx = new AnalogGyro(Portmap.GYRO);

    // This assumes the robot will start backwards at the beginning of the match.
    // navx.setAngleAdjustment(180);

    rightEncoder = new Encoder(Portmap.RIGHT_ENCODER_1, Portmap.RIGHT_ENCODER_2, false);
    leftEncoder = new Encoder(Portmap.LEFT_ENCODER_1, Portmap.LEFT_ENCODER_2, true);

    rightDriveMotorTalon = new WPI_TalonSRX(Portmap.RIGHTDRIVETALON);
    leftDriveMotorTalon = new WPI_TalonSRX(Portmap.LEFTDRIVETALON);
    if (practice) {
      rightDriveMotorVictor = new WPI_TalonSRX(Portmap.RIGHTDRIVEVICTOR);
      leftDriveMotorVictor = new WPI_TalonSRX(Portmap.LEFTDRIVEVICTOR);
    }
    else {
      rightDriveMotorVictor = new VictorSPX(Portmap.RIGHTDRIVEVICTOR);
      leftDriveMotorVictor = new VictorSPX(Portmap.LEFTDRIVEVICTOR);
    }
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

    leftEncoder.setDistancePerPulse(Constants.distancePerPulse);
    rightEncoder.setDistancePerPulse(Constants.distancePerPulse);

    navx.calibrate();
    resetEncoders();

    drive = new DifferentialDrive(leftDriveMotorTalon, rightDriveMotorTalon);

    odometry = new DifferentialDriveOdometry(getAngleRot2d());
  }

  @Override
  public void periodic() {
    updateSmartDashboard();

    odometry.update(getAngleRot2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
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
    Sendable pdp = Robot.PDP;
    // SmartDashboard.putBoolean("NAVX CONNECTED", navx.isConnected());
    SmartDashboard.putData("Gyro", dataForGyro);
    SmartDashboard.putData("PDP", pdp);
    SmartDashboard.putNumber("Heading", navx.getAngle());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.get());
    SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
    SmartDashboard.putNumber("Odometry x", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odomtry.y", odometry.getPoseMeters().getY());
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
    correctionFactor = (error / 75.0);

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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public Rotation2d getAngleRot2d() {
    return navx.getRotation2d();
  } 

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftDriveMotorTalon.setVoltage(-leftVolts);
    rightDriveMotorTalon.setVoltage(rightVolts);
    SmartDashboard.putNumber("Drivetrain.LeftVolts", leftVolts);
    SmartDashboard.putNumber("DriveTrain.rightVolt", -rightVolts);
    drive.feed();
  }
}
