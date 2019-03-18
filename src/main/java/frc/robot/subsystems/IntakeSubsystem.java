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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.commands.HatchGrabberCMDS;

/** An example subsystem. You can replace me with your own Subsystem. */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DoubleSolenoid grabber = new DoubleSolenoid(Portmap.GRABBER_RELEASED, Portmap.GRABBER_GRAB);
  public DoubleSolenoid retractor = new DoubleSolenoid(Portmap.GRABBER_STOWED, Portmap.GRABBER_UP);
  public Compressor compressor = new Compressor();
  private DigitalInput hatchLimit1 = new DigitalInput(Portmap.HATCH_SENSOR_SWITCH1);
  private DigitalInput hatchLimit2 = new DigitalInput(Portmap.HATCH_SENSOR_SWITCH2);

  public IntakeSubsystem() {

  }

  public void autoInit() {

  }

  @Override
  public void periodic() {

    /*
     * kF = SmartDashboard.getNumber("F", 0); kP = SmartDashboard.getNumber("P", 0);
     * kI = SmartDashboard.getNumber("I", 0); kD = SmartDashboard.getNumber("D", 0);
     * 
     * boolean resetEncoder = SmartDashboard.getBoolean("ResetEncoder", false);
     * 
     * 
     * rightIntakeMotor.config_kF(loopID, kF, timeoutMS);
     * rightIntakeMotor.config_kP(loopID, kP, timeoutMS);
     * rightIntakeMotor.config_kD(loopID, kD, timeoutMS);
     * rightIntakeMotor.config_kI(loopID, kI, timeoutMS);
     * 
     * if(resetEncoder){ rightIntakeMotor.setSelectedSensorPosition(0, loopID, 30);
     * }
     * 
     */
    printTelemetry();
  }

  /*
   * public void unStow() { if (!stowed) { return; } double targetPosition =
   * .17055555 * gearRatio * 4096; rightIntakeMotor.set(ControlMode.Position,
   * targetPosition); stowed = false; }
   */
  public void stow() {
    releaseHatch();
    retractor.set(Value.kForward);
  }

  public void moveGrabberUp() {
    retractor.set(Value.kReverse);
  }

  public void releaseHatch() {
    grabber.set(Value.kForward);
  }

  public void grabHatch() {
    grabber.set(Value.kReverse);

  }

  public boolean armGrabber() {
    if (getHatchSwitches()) {
      grabHatch();
      return true;
    } else {
      return false;
    }
  }

  public boolean getHatchSwitches() {
    // This might have to be reversed.
    return (hatchLimit1.get() || hatchLimit2.get());
  }

  public void stop() {

  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HatchGrabberCMDS.ManualGrabCMD());
  }

  public void printTelemetry() {
    SmartDashboard.putBoolean("Hatch switches", getHatchSwitches());
  }
}
