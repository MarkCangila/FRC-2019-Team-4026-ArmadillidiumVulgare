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
  Timer updateTimer = new Timer();

  public HatchLocation(double timeOut) {
    timeOutMax = timeOut;
  }

  public String toString() {
    return (String) (identifier + ": " + distance + "ft, " + getAngle() + " dgrs");
  }

  public boolean isReal() {
    return !(distance == -100 && angle == -100);
  }
  // This will set the angle and reset the timer if the angle is deemed valid (!= -100)
  public void updateAngle(double angleVal) {
    if (angleVal != -100) {
      angle = angleVal;
      updateTimer.reset();
      updateTimer.start();
    }
  }
  // Will return the last known angle of the hatch if the timeout is not expired.
  public double getAngle() {
    if (updateTimer.get() < timeOutMax) {
      return angle;
    } else {
      return -100;
    }
  }
}
