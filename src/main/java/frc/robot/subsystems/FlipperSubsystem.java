package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.commands.FlipperCMDS;

public class FlipperSubsystem extends Subsystem {
  public WPI_TalonSRX rightFlipper = new WPI_TalonSRX(Portmap.RIGHTFLIPPER);
  public WPI_TalonSRX leftFlipper = new WPI_TalonSRX(Portmap.LEFTFLIPPER);

  private final int timeoutMS = 30;
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

    rightFlipper.setInverted(rightInverted);
    leftFlipper.setInverted(leftInverted);

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

    int absolutePositionRight = rightFlipper.getSensorCollection().getPulseWidthPosition();
    int absolutePositionLeft = leftFlipper.getSensorCollection().getPulseWidthPosition();

    absolutePositionLeft &= 0xFFFFFF;
    absolutePositionRight &= 0xFFFFFF;

    if (rightSensorPhase) {
      absolutePositionRight *= -1;
    }
    if (leftSensorPhase) {
      absolutePositionLeft *= -1;
    }
    if (rightInverted) {
      absolutePositionRight *= -1;
    }
    if (leftInverted) {
      absolutePositionLeft *= -1;
    }

    rightFlipper.setSelectedSensorPosition(absolutePositionRight, loopID, timeoutMS);
    leftFlipper.setSelectedSensorPosition(absolutePositionLeft, loopID, timeoutMS);
    rightFlipper.setNeutralMode(NeutralMode.Brake);
    leftFlipper.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new FlipperCMDS.ManualFlip());
  }

  public void printTelemetry() {
    SmartDashboard.putNumber("Encoder Value Right", rightFlipper.getSelectedSensorPosition(loopID));
    SmartDashboard.putNumber("Encoder Value Left", leftFlipper.getSelectedSensorPosition(loopID));
    SmartDashboard.putNumber("Speed Right", rightFlipper.get());
    SmartDashboard.putNumber("Speed Left", leftFlipper.get());
  }
}
