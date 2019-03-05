/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class HatchLocation {
  private double timeOutMax;
  private double distance;
  private double angle = -100;
  public double otherAngle;
  public String identifier;
  public int lastUpdated;
  private double targetHeading = -100;
  Timer updateTimer = new Timer();

  public HatchLocation(double timeOut) {
    timeOutMax = timeOut;
  }

  public String toString() {
    return (String) (identifier + ": " + distance + "ft, " + getAngleDeg() + " dgrs");
  }

  public boolean isReal() {
    return !(getAngleRad() == -100);
  }
  // This will set the angle and reset the timer if the angle is deemed valid (!= -100)
  public void updateAngle(double angleVal) {
    if (angleVal != -100) {
      angle = angleVal;
      updateTimer.reset();
      updateTimer.start();
      targetHeading = angle + Robot.driveTrainSubsystem.getAngle();
    }
  }
  // Will return the last known angle of the hatch if the timeout is not expired.
  public double getAngleRad() {
    if (updateTimer.get() < timeOutMax) {
      return angle;
    } else {
      return -100;
    }
  }

  public double getAngleDeg() {
    return getAngleRad() * (180 / Math.PI);
  }

  public double getTargetHeading() {
    if (updateTimer.get() < timeOutMax) {
      return targetHeading;
    } else {
      return -100;
    }
  }
}
