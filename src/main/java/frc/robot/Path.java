package frc.robot;

import jaci.pathfinder.Trajectory;

public class Path {
    public Trajectory leftTrajectory;
    public Trajectory rightTrajectory;
    
    public Path(Trajectory left, Trajectory right) {
        leftTrajectory = left;
        rightTrajectory = right;
    }
}