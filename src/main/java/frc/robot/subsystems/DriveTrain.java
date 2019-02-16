package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.kauailabs.navx.frc.AHRS;

public abstract class DriveTrain extends Subsystem { 
    public abstract void leftPower(double requestedPower);

    public abstract void rightPower(double requestedPower);

    public abstract void periodic();

    protected abstract void initDefaultCommand();

    public abstract void stop();

    public abstract void keepDriveStraight(double leftDriveVel, double rightDriveVel, double targetAngle);

    public abstract void dumbDriveStraight(double power);

    public abstract double getAngle();

    public AHRS navx;
}