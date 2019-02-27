/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CenterAuto;
import frc.robot.commands.DriveTrainCMDS;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FlipperCMDS;
import frc.robot.commands.HatchGrabberCMDS;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import frc.robot.subsystems.DriveTrainSubsystem2019;
import frc.robot.subsystems.DriveTrainSubsystemPractice;
import frc.robot.subsystems.FlipperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionSystem;
//import frc.robot.subsystems.GyroSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final boolean practice = true;

  public static DriveTrainSubsystem2019 driveTrainSubsystem = new DriveTrainSubsystem2019(practice);
  public static IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // public static DriveTrainSubsystem2018 driveTrainSubsystem = new DriveTrainSubsystem2018();
  //public static DriveTrainSubsystem2019 driveTrainSubsystem = new DriveTrainSubsystem2019();
  public static VisionSystem visionSystem = new VisionSystem();
  public static PowerDistributionPanel PDP = new PowerDistributionPanel(0);
  public static FlipperSubsystem flipperSubsystem = new FlipperSubsystem();
  public static BuiltInAccelerometer Accelerometer = new BuiltInAccelerometer(Range.k8G);
  
  
  public static OI oi;
  

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    oi = new OI();
    oi.stick2Button1.whileHeld(new HatchGrabberCMDS.GoDownCMD());
    oi.stick2Button2.whileHeld(new HatchGrabberCMDS.StowCMD());
    oi.stick2Button4.whileHeld(new HatchGrabberCMDS.AlmostDownCMD());
    oi.stick2Button3.whileHeld(new HatchGrabberCMDS.GoUpCMD());
    oi.stick1Button8.whileHeld(new DriveTrainCMDS.DriveStraight());
    oi.stick2Button8.whileHeld(new HatchGrabberCMDS.Eject());
    oi.stick2Button9.whileHeld(new FlipperCMDS.AutoFlip());
    oi.stick1Button6.whileHeld(new DriveTrainCMDS.DriveToRightHatchCMD());
    m_chooser.setDefaultOption("Default Auto", new CenterAuto());
    m_chooser.setDefaultOption("Left Far Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    //robotChooser.setDefaultOption("Main Bot", new DriveTrainSubsystem2019());
    //robotChooser.addOption("Pratice Bot", new DriveTrainSubsystemPractice());
    SmartDashboard.putData("Auto mode", m_chooser);
    //SmartDashboard.putData("Robot type", robotChooser);
    //CameraServer.getInstance().startAutomaticCapture();

    //driveTrainSubsystem = new DriveTrainSubsystem2019();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This function is called once each time the robot enters Disabled mode. You can use it to reset
   * any subsystem information you want to clear when the robot is disabled.
   */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the chooser code above
   * (like the commented example) or additional comparisons to the switch structure below with
   * additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    intakeSubsystem.autoInit();
    m_autonomousCommand = m_chooser.getSelected();
    driveTrainSubsystem.resetEncoders();
    
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
   // System.out.println(intakeSubsystem.rightIntakeMotor.getSelectedSensorVelocity(0));
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    flipperSubsystem.initEnable();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
