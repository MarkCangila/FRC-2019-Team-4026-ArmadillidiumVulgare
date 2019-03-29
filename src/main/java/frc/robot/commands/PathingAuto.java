package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class PathingAuto extends CommandGroup {
    public PathingAuto() {
        addParallel(new HatchGrabberCMDS.GoUpCMD());
        addSequential(new DriveTrainCMDS.FollowPath(Robot.paths.get("CenterToHatch")));
        addParallel()
    }
}