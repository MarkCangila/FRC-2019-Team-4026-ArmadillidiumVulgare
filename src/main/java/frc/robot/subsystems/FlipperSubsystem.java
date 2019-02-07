package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.commands.FlipperCMDS;

public class FlipperSubsystem extends Subsystem {
  double flipperPower = 0;

  //New rule, the Command class is no longer allowed to speak directly to the motor controllers. 
  //It must change power by calling the setFlipperPower method.
  private WPI_TalonSRX rightFlipper = new WPI_TalonSRX(Portmap.RIGHTFLIPPER);
  private WPI_TalonSRX leftFlipper = new WPI_TalonSRX(Portmap.LEFTFLIPPER);

  private final int timeoutMS = 30;
  private final int currentLimit = 60; //Max current in amps
  private final int currentDuration = 30;
  public final int softLimitForAutoFlip = 0; //Set this to the encoder value recorded when the robot tips.
  private final double kF = 0;
  private final double kP = 0;
  private final double kD = 0;
  private final double kI = 0;
  private final boolean rightSensorPhase = true;
  private final boolean leftSensorPhase = true;
  private final boolean rightInverted = false;
  private final boolean leftInverted = false;
  private final int loopID = 0;

  public FlipperSubsystem() {
    rightFlipper.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative, loopID, timeoutMS);
    leftFlipper.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative, loopID, timeoutMS);

    rightFlipper.setSensorPhase(rightSensorPhase);
    leftFlipper.setSensorPhase(leftSensorPhase);

    
/*
    rightFlipper.configNominalOutputForward(0, 30);
    rightFlipper.configNominalOutputReverse(0, 30);
    leftFlipper.configNominalOutputForward(0, 30);
    leftFlipper.configNominalOutputReverse(0, 30);

    rightFlipper.configPeakOutputForward(1, 30);
    rightFlipper.configPeakOutputReverse(-1, 30);
    leftFlipper.configPeakOutputForward(1, 30);
    leftFlipper.configPeakOutputReverse(-1, 30);

    rightFlipper.configAllowableClosedloopError(0, loopID, timeoutMS);
    leftFlipper.configAllowableClosedloopError(0, loopID, timeoutMS);

    rightFlipper.config_kF(loopID, kF, timeoutMS);
    rightFlipper.config_kP(loopID, kP, timeoutMS);
    rightFlipper.config_kD(loopID, kD, timeoutMS);
    rightFlipper.config_kI(loopID, kI, timeoutMS);
    leftFlipper.config_kF(loopID, kF, timeoutMS);
    leftFlipper.config_kP(loopID, kP, timeoutMS);
    leftFlipper.config_kD(loopID, kD, timeoutMS);
    leftFlipper.config_kI(loopID, kI, timeoutMS);
      */
    int absolutePositionRight = rightFlipper.getSensorCollection().getPulseWidthPosition();
    int absolutePositionLeft = leftFlipper.getSensorCollection().getPulseWidthPosition();

    
    leftFlipper.setInverted(true);


    leftFlipper.configPeakCurrentLimit(currentLimit);
    leftFlipper.configPeakCurrentDuration(currentDuration);
    leftFlipper.configContinuousCurrentLimit(60);

    rightFlipper.configPeakCurrentLimit(currentLimit);
    rightFlipper.configPeakCurrentDuration(currentDuration);
    rightFlipper.configContinuousCurrentLimit(60);

    absolutePositionLeft &= 0xFFFFFF;
    absolutePositionRight &= 0xFFFFFF;

    rightFlipper.setSelectedSensorPosition(absolutePositionRight, loopID, timeoutMS);
    leftFlipper.setSelectedSensorPosition(absolutePositionLeft, loopID, timeoutMS);
    rightFlipper.setNeutralMode(NeutralMode.Brake);
    leftFlipper.setNeutralMode(NeutralMode.Brake);


  }



  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new FlipperCMDS.ManualFlip());
  
  }

  @Override
  public void periodic() {
    updateMotorControllers();
    printTelemetry();
    super.periodic();
  }

  public void setFlipperPower(double power){
    flipperPower = power;
  }

  public void stopFlipper(){
    setFlipperPower(0);
  }

  public int getPosition(){
    //Right now only the right flipper has it's encoder connected
   return rightFlipper.getSelectedSensorPosition();
  }

  private void updateMotorControllers(){
    rightFlipper.set(flipperPower);
    leftFlipper.set(flipperPower);
  }


  

  public void printTelemetry() {
    SmartDashboard.putNumber("Encoder Value Right", rightFlipper.getSelectedSensorPosition(loopID));
    SmartDashboard.putNumber("Encoder Value Left", leftFlipper.getSelectedSensorPosition(loopID));
    SmartDashboard.putNumber("Speed Right", rightFlipper.get());
    SmartDashboard.putNumber("Speed Left", leftFlipper.get());
  }
}
