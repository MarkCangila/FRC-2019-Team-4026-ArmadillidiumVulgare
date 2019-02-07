/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Portmap;
import frc.robot.commands.HatchGrabberCMDS;

/** An example subsystem. You can replace me with your own Subsystem. */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX rightIntakeMotor = new WPI_TalonSRX(Portmap.HATCHGRABBER);

  private final int gearRatio = 1;
  private final int timeoutMS = 30;
  private final double kF = 0;
  private final double kP = 0;
  private final double kD = 0;
  private final double kI = 0;
  private final boolean sensorPhase = true;
  private final boolean inverted = false;
  private boolean atGround = false;
  public boolean stowed = false;
  private final int loopID = 0;

  public IntakeSubsystem() {
    rightIntakeMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeMotor.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative, loopID, timeoutMS);
    rightIntakeMotor.setSensorPhase(sensorPhase);
    rightIntakeMotor.setInverted(inverted);

    rightIntakeMotor.configNominalOutputForward(0, 30);
    rightIntakeMotor.configNominalOutputReverse(0, 30);

    rightIntakeMotor.configPeakOutputForward(1, 30);
    rightIntakeMotor.configPeakOutputReverse(-1, 30);

    rightIntakeMotor.configAllowableClosedloopError(0, loopID, timeoutMS);

    rightIntakeMotor.config_kF(loopID, kF, timeoutMS);
    rightIntakeMotor.config_kP(loopID, kP, timeoutMS);
    rightIntakeMotor.config_kD(loopID, kD, timeoutMS);
    rightIntakeMotor.config_kI(loopID, kI, timeoutMS);

    int absolutePosition = rightIntakeMotor.getSensorCollection().getPulseWidthPosition();

    absolutePosition &= 0xFFFFFF;

    if (sensorPhase) {
      absolutePosition *= -1;
    }
    if (inverted) {
      absolutePosition *= -1;
    }

    rightIntakeMotor.setSelectedSensorPosition(absolutePosition, loopID, timeoutMS);
  }

  public void goDown() {
    if (atGround || stowed) {
      return;
    }
    double targetPosition = 0.25 * gearRatio * 4096;
    rightIntakeMotor.set(ControlMode.Position, targetPosition);
    atGround = true;
  }

  public void goUp() {
    if (!atGround) {
      return;
    }
    double targetPosition = -0.25 * gearRatio * 4096;
    rightIntakeMotor.set(ControlMode.Position, targetPosition);
    atGround = false;
  }

  public void unStow() {
    if (!stowed) {
      return;
    }
    double targetPosition = .17055555 * gearRatio * 4096;
    rightIntakeMotor.set(ControlMode.Position, targetPosition);
    stowed = false;
  }

  public void stow() {
    if (atGround || stowed) {
      return;
    }
    double targetPosition = -.17055555 * gearRatio * 4096;
    rightIntakeMotor.set(ControlMode.Position, targetPosition);
    stowed = true;
  }

  public void intake() {
    rightIntakeMotor.set(.5);
  }

  public void outtake() {
    rightIntakeMotor.set(-.5);
  }

  public void stop() {
    rightIntakeMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HatchGrabberCMDS.StopCMD());
  }
}
