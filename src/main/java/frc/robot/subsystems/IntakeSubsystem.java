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

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.commands.HatchGrabberCMDS;

/** An example subsystem. You can replace me with your own Subsystem. */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX rightIntakeMotor = new WPI_TalonSRX(Portmap.HATCHGRABBER);
  public DoubleSolenoid ejector = new DoubleSolenoid(Portmap.EJECTOR_OUT, Portmap.EJECTOR_IN);
  public Compressor compressor = new Compressor();
  private DigitalInput reverseLimit = new DigitalInput(Portmap.HATCH_REVERSE_LIMIT);
  private final int STOW_POSITION = 0;
  private final int UP_POSITION = 1850; //CHANGE THIS
  private final int DOWN_POSITION = 3800; //CHange this
  private final int ALMOST_DOWN_POSITION = 1850;

  private final int gearRatio = 1;
  private final int timeoutMS = 30;
  private double kF = 0;
  private double kP = .45;
  private double kD = .1;
  private double kI = 0.0001;
  private final boolean sensorPhase = true;
  private final boolean inverted = false;
  private boolean atGround = false;
  public boolean stowed = false;
  private final int loopID = 0;

  public IntakeSubsystem() {
    ejector.set(DoubleSolenoid.Value.kReverse);
    rightIntakeMotor.setNeutralMode(NeutralMode.Brake);
    rightIntakeMotor.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative, loopID, timeoutMS);
    rightIntakeMotor.setSensorPhase(sensorPhase);
    rightIntakeMotor.setInverted(inverted);

    rightIntakeMotor.set(ControlMode.Position, 0);

    rightIntakeMotor.configNominalOutputForward(0, 30);
    rightIntakeMotor.configNominalOutputReverse(0, 30);

    rightIntakeMotor.configPeakOutputForward(1, 30);
    rightIntakeMotor.configPeakOutputReverse(-1, 30);

    rightIntakeMotor.configAllowableClosedloopError(0, loopID, timeoutMS);

    rightIntakeMotor.config_kF(loopID, kF, timeoutMS);
    rightIntakeMotor.config_kP(loopID, kP, timeoutMS);
    rightIntakeMotor.config_kD(loopID, kD, timeoutMS);
    rightIntakeMotor.config_kI(loopID, kI, timeoutMS);
    rightIntakeMotor.configForwardSoftLimitEnable(false);
    rightIntakeMotor.configReverseSoftLimitEnable(false);
    

    int absolutePosition = rightIntakeMotor.getSensorCollection().getPulseWidthPosition();

    absolutePosition &= 0xFFFFFF;

    if (sensorPhase) {
      absolutePosition *= -1;
    }
    if (inverted) {
      absolutePosition *= -1;
    }

    //rightIntakeMotor.setSelectedSensorPosition(0, loopID, timeoutMS);
  }
  @Override
  public void periodic(){
  
    /*
    kF = SmartDashboard.getNumber("F", 0);
    kP = SmartDashboard.getNumber("P", 0);
    kI = SmartDashboard.getNumber("I", 0);
    kD = SmartDashboard.getNumber("D", 0);

    boolean resetEncoder = SmartDashboard.getBoolean("ResetEncoder", false);


    rightIntakeMotor.config_kF(loopID, kF, timeoutMS);
    rightIntakeMotor.config_kP(loopID, kP, timeoutMS);
    rightIntakeMotor.config_kD(loopID, kD, timeoutMS);
    rightIntakeMotor.config_kI(loopID, kI, timeoutMS);

    if(resetEncoder){
      rightIntakeMotor.setSelectedSensorPosition(0, loopID, 30);
    }
    
    */
    printTelemetry();
  }
  
  public void goDown() {
    
      double targetPosition = DOWN_POSITION;
      rightIntakeMotor.set(ControlMode.Position, targetPosition);
     // atGround = true;
    
  }

  public void goAlmostDown(){
    double targetPosition = ALMOST_DOWN_POSITION;
    rightIntakeMotor.set(ControlMode.Position, targetPosition);

  }



  public void goUp() {
  
    double targetPosition = UP_POSITION;
    rightIntakeMotor.set(ControlMode.Position, targetPosition);
    atGround = false;
  }


/*
  public void unStow() {
    if (!stowed) {
      return;
    }
    double targetPosition = .17055555 * gearRatio * 4096;
    rightIntakeMotor.set(ControlMode.Position, targetPosition);
    stowed = false;
  }
*/
  public void stow() {
    
    double targetPosition = STOW_POSITION;
    if (reverseLimit.get()){
      rightIntakeMotor.set(ControlMode.PercentOutput, -.4);

    }else{
      rightIntakeMotor.set(ControlMode.PercentOutput, 0);
      rightIntakeMotor.setSelectedSensorPosition(0, loopID, timeoutMS);

    }

    
  }

  public void stop() {
    rightIntakeMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new HatchGrabberCMDS.ManualControlCMD());
  }
  

  public void printTelemetry(){
    SmartDashboard.putNumber("Encoder Value Hatch", rightIntakeMotor.getSelectedSensorPosition());
   // SmartDashboard.putNumber("Target Hatch", rightIntakeMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("Error Hatch", rightIntakeMotor.getClosedLoopError());
  }
}
