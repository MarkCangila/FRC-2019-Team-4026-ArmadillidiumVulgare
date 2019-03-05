package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.commands.FlipperCMDS;

public class FlipperSubsystem extends Subsystem {
  double flipperPower = 0;

  // New rule, the Command class is no longer allowed to speak directly to the motor controllers.
  // It must change power by calling the setFlipperPower method.
  private WPI_TalonSRX rightFlipper = new WPI_TalonSRX(Portmap.RIGHTFLIPPER);
  private WPI_TalonSRX leftFlipper = new WPI_TalonSRX(Portmap.LEFTFLIPPER);

  private DigitalInput reverseLimitSwitch = new DigitalInput(Portmap.FLIPPER_REVERSE_LIMIT);
  private DigitalInput forewardLimitSwitch = new DigitalInput(Portmap.FLIPPER_FOREWARD_LIMIT);

  private final int timeoutMS = 30;
  private final int currentLimit = 60; // Max current in amps
  private final int currentDuration = 350;
  public final int softLimitForAutoFlip =
      0; // Set this to the encoder value recorded when the robot tips.
  private static final double ACCEPTABLE_ERROR = 100;
  private static final int FOREWARD_LIMIT_ENC = 4000; // CHANGE THIS

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

    leftFlipper.configForwardSoftLimitThreshold(FOREWARD_LIMIT_ENC);
    rightFlipper.configForwardSoftLimitThreshold(FOREWARD_LIMIT_ENC);

    leftFlipper.configForwardSoftLimitEnable(false);
    rightFlipper.configForwardSoftLimitEnable(false);

    leftFlipper.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative, loopID, timeoutMS);
    leftFlipper.setSensorPhase(true);

    rightFlipper.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative, loopID, timeoutMS);
    rightFlipper.setSensorPhase(true);

    rightFlipper.setInverted(true);
    leftFlipper.setInverted(false);

    leftFlipper.configPeakCurrentLimit(currentLimit);
    leftFlipper.configPeakCurrentDuration(currentDuration);
    leftFlipper.configContinuousCurrentLimit(65);

    rightFlipper.configPeakCurrentLimit(currentLimit);
    rightFlipper.configPeakCurrentDuration(currentDuration);
    rightFlipper.configContinuousCurrentLimit(65);

    rightFlipper.setSelectedSensorPosition(0, loopID, timeoutMS);
    leftFlipper.setSelectedSensorPosition(0, loopID, timeoutMS);

    rightFlipper.setNeutralMode(NeutralMode.Brake);
    leftFlipper.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new FlipperCMDS.ManualFlip());
  }

  public void initEnable() {
    rightFlipper.setSelectedSensorPosition(0, loopID, timeoutMS);
    leftFlipper.setSelectedSensorPosition(0, loopID, timeoutMS);
  }

  @Override
  public void periodic() {
    updateMotorControllers();
    printTelemetry();
    // checkLimitSwitches();
    super.periodic();
  }

  public void setFlipperPower(double power) {
    flipperPower = power;
  }

  public void stopFlipper() {
    setFlipperPower(0);
  }

  public int getPosition() {
    // Right now only the right flipper has it's encoder connected
    return rightFlipper.getSelectedSensorPosition();
  }

  private void updateMotorControllers() {
    double rightPos = Math.abs(rightFlipper.getSelectedSensorPosition(loopID));
    double leftPos = Math.abs(leftFlipper.getSelectedSensorPosition(loopID));
    double rightPower = flipperPower;
    double leftPower = flipperPower;

    /*
        if(flipperPower > 0){
          if (rightPos - leftPos > ACCEPTABLE_ERROR){
            leftPower = 0;
          } else if (rightPos - leftPos < ACCEPTABLE_ERROR){
            rightPower = 0;
          }
        }else if (flipperPower < 0){
          if (rightPos - leftPos > ACCEPTABLE_ERROR){
            rightPower = 0;
          } else if (rightPos - leftPos < ACCEPTABLE_ERROR){
            leftPower = 0;
          }
        }
    */
    if (flipperPower < 0 && reverseLimitSwitch.get()) {
      rightFlipper.setSelectedSensorPosition(0, loopID, timeoutMS);
      leftFlipper.setSelectedSensorPosition(0, loopID, timeoutMS);
      rightPower = 0;
      leftPower = 0;
    }

    if (flipperPower > 0 && forewardLimitSwitch.get()) {
      rightPower = 0;
      leftPower = 0;
    }

    rightFlipper.set(rightPower);
    leftFlipper.set(leftPower);
  }

  private void checkLimitSwitches() {
    if (flipperPower < 0 && reverseLimitSwitch.get()) {
      rightFlipper.setSelectedSensorPosition(0, loopID, timeoutMS);
      leftFlipper.setSelectedSensorPosition(0, loopID, timeoutMS);
      flipperPower = 0;
    }

    if (flipperPower < 0 && forewardLimitSwitch.get()) {
      flipperPower = 0;
    }
  }

  public void printTelemetry() {
    SmartDashboard.putNumber("Encoder Value Right", rightFlipper.getSelectedSensorPosition(loopID));
    SmartDashboard.putNumber("Encoder Value Left", leftFlipper.getSelectedSensorPosition(loopID));
    SmartDashboard.putNumber("Speed Right", rightFlipper.get());
    SmartDashboard.putNumber("Speed Left", leftFlipper.get());
    SmartDashboard.putBoolean("Reverse Limit", reverseLimitSwitch.get());
    SmartDashboard.putBoolean("Front Limit", forewardLimitSwitch.get());
  }
}
