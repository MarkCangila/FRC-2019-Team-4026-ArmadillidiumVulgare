/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CenterAuto extends CommandGroup {
  /** Add your docs here. */
  public CenterAuto() {
    addParallel(new HatchGrabberCMDS.GoUpCMD());
    addSequential(new DriveTrainCMDS.DriveStraightDist(40, .5, .5, 0));
    addSequential(new DriveTrainCMDS.DriveStraightDist(93, .9, .2, 0));
    addParallel(new HatchGrabberCMDS.Eject());
    addSequential(new DriveTrainCMDS.DriveStraightDist(-10, -.2, -.2, 0));
  }
}
