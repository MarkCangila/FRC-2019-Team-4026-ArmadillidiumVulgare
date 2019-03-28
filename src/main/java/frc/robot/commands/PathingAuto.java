package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class PathingAuto extends CommandGroup {
    public PathingAuto() {
        addParallel(new HatchGrabberCMDS.GoUpCMD());
        addSequential(new DriveTrainCMDS.FollowPath(Robot.paths.get("CenterToHatch")));
        addSequential(new DriveTrainCMDS.AutoLineUpToNinety(.025, 0, 0));
        addSequential(new HatchGrabberCMDS.Eject());
        addSequential(new DriveTrainCMDS.Turn(180, false));
        addParallel(new DriveTrainCMDS.FollowPath(Robot.paths.get("HatchToReload")));
        addSequential(new DriveTrainCMDS.AutoLineUpToNinety(.025, 0, 0));
        addSequential(new DriveTrainCMDS.Turn(180, false));
        addParallel(new HatchGrabberCMDS.Eject());
    }
}