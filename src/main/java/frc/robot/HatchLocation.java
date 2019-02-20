/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */


public class HatchLocation {
    public double distance;
    public double angle;
    public double otherAngle;
    public String identifier;
    
    
public HatchLocation(){}

    public String toString(){
        return (String) (identifier + ": " + distance + "ft, " + angle + " dgrs");
    }

    public boolean isReal(){
        return !(distance == -100 && angle == -100);
        
    }
}
