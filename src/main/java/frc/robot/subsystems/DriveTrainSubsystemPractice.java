package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SerialPort;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.commands.DriveTrainCMDS;

public class DriveTrainSubsystemPractice extends DriveTrain {

  final WPI_TalonSRX rightDriveMotorTalon;
  final WPI_TalonSRX leftDriveMotorTalon;
  final WPI_TalonSRX rightDriveMotorTalon2;
  final WPI_TalonSRX leftDriveMotorTalon2;

  Encoder rightEncoder;
  Encoder leftEncoder;
  
  static final double MAXPOWERCHANGE = .1;
  
  public AHRS navx;

  
  public void dumbDriveStraight(double power) {
    
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveTrainCMDS.TankDrive());
  }

  public DriveTrainSubsystemPractice() {

    navx = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte)50);

    //This assumes the robot will start backwards at the beginning of the match.
    navx.setAngleAdjustment(180);

    rightEncoder = new Encoder(Portmap.RIGHT_ENCODER_1, Portmap.RIGHT_ENCODER_2, false);
		leftEncoder = new Encoder(Portmap.LEFT_ENCODER_1, Portmap.LEFT_ENCODER_2, true);
    rightDriveMotorTalon = new WPI_TalonSRX(Portmap.LEFTDRIVETALON); 
    leftDriveMotorTalon = new WPI_TalonSRX(Portmap.RIGHTDRIVETALON);
    rightDriveMotorTalon2 = new WPI_TalonSRX(Portmap.LEFTDRIVEVICTOR);
    leftDriveMotorTalon2 = new WPI_TalonSRX(Portmap.RIGHTDRIVEVICTOR);
    // rightDriveMotorVictor.follow(rightDriveMotorTalon);
    // leftDriveMotorVictor.follow(leftDriveMotorTalon);
    // rightDriveMotorTalon.setInverted(true);
    rightDriveMotorTalon.setNeutralMode(NeutralMode.Brake);
    leftDriveMotorTalon.setNeutralMode(NeutralMode.Brake);
    rightDriveMotorTalon2.setNeutralMode(NeutralMode.Brake);
    leftDriveMotorTalon2.setNeutralMode(NeutralMode.Brake);
    rightDriveMotorTalon.setInverted(true);
    rightDriveMotorTalon2.setInverted(true);
    rightDriveMotorTalon2.follow(rightDriveMotorTalon);
    leftDriveMotorTalon2.follow(leftDriveMotorTalon);
  }

  @Override
  public void periodic() {
    updateSmartDashboard();
  }

  public void leftPower(double requestedPower) {
    //requestedPower = requestedPower;
    double currentPower = leftDriveMotorTalon.get();
    double newPower;
    if (requestedPower < currentPower) {
      newPower = Math.max(requestedPower, currentPower - MAXPOWERCHANGE);
    } else if (requestedPower > currentPower) {
      newPower = Math.min(requestedPower, currentPower + MAXPOWERCHANGE);
    } else {
      newPower = requestedPower;
    }
    //System.out.println(
      //  String.format(
        //    "Left:requestedPower: %.2f, resultingPower: %.2f, currentPower: %.2f",
          //  requestedPower, newPower, currentPower));
    leftDriveMotorTalon.set(newPower);
  }

  public void rightPower(double requestedPower) {
    //requestedPower = requestedPower;
    double currentPower = rightDriveMotorTalon.get();
    double newPower;
    if (requestedPower < currentPower) {
      newPower = Math.max(requestedPower, currentPower - MAXPOWERCHANGE);
    } else if (requestedPower > currentPower) {
      newPower = Math.min(requestedPower, currentPower + MAXPOWERCHANGE);
    } else {
      newPower = requestedPower;
    }
    //System.out.println(
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

  private void updateSmartDashboard(){
    Sendable dataForGyro = navx;

    SmartDashboard.putBoolean("NAVX CONNECTED", navx.isConnected());
    SmartDashboard.putData("Gyro", dataForGyro);
    SmartDashboard.putNumber("Heading", navx.getYaw());
  }





  public void keepDriveStraight(double leftDriveVel, double rightDriveVel, double targetAngle) {

		double error = 0, correctionFactor;
		error = targetAngle + navx.getAngle();
		correctionFactor = (error / 75.0);

		// todo - best practice - conditions on a separate line should be
		// wrapped in brackets
		if (leftDriveVel > 0.9)
			leftDriveVel = 0.9;
		else if (leftDriveVel < -0.9)
			leftDriveVel = -0.9;

		// todo - best practice - conditions on a separate line should be
		// wrapped in brackets
		if (rightDriveVel > 0.9)
			rightDriveVel = 0.9;
		else if (rightDriveVel < -0.9)
			rightDriveVel = -0.9;

		if (targetAngle > (navx.getAngle() - 0.5) || targetAngle < (navx.getAngle() + 0.5)) {
			rightPower(((leftDriveVel) - correctionFactor));
			leftPower((rightDriveVel + correctionFactor));
		} else {
			rightPower(leftDriveVel);
			leftPower(rightDriveVel);
		}
  }
  
  public double getAngle(){
   return navx.getAngle();
  }

  public int getEncoderRight(){
    return rightEncoder.get();
  }

  public int getEncoderLeft(){
    return leftEncoder.get();
  }

  public void resetEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();

  }

}
