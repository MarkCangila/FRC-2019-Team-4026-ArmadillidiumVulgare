/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Portmap;
import frc.robot.commands.ShooterCMDS;


/** An example subsystem. You can replace me with your own Subsystem. */
public class IntakeSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Servo ballFeeder = new Servo(Portmap.BALLSERVO);
  public WPI_TalonSRX backIntakeMotor = new WPI_TalonSRX(Portmap.BACKINTAKEMOTOR);
  public WPI_TalonSRX frontIntakeMotor = new WPI_TalonSRX(Portmap.FRONTINTAKEMOTOR);
  
  public boolean isOpen = false;

  public IntakeSubsystem() {
    backIntakeMotor.setNeutralMode(NeutralMode.Coast);
    frontIntakeMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void autoInit() {}

  @Override
  public void periodic() {
    printTelemetry();
  }

  public void stop() {
    backIntakeMotor.set(0);
    frontIntakeMotor.set(0);
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ShooterCMDS.Stop());
  }

  public void printTelemetry() {
    SmartDashboard.putNumber("Encoder Value Hatch", frontIntakeMotor.getSelectedSensorPosition());
    // SmartDashboard.putNumber("Target Hatch", rightIntakeMotor.getClosedLoopTarget());
    SmartDashboard.putNumber("Error Hatch", frontIntakeMotor.getClosedLoopError());
  }
}
