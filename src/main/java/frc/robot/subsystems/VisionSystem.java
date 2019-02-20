/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.HatchLocation;

/**
 * Add your docs here.
 */

public class VisionSystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final int MAX_HATCH_COUNT = 2;
  private HatchLocation hatch1;
  private HatchLocation hatch2;
  NetworkTable table;
  String[] toStrings;
  Boolean inited = false;
  NetworkTableEntry h1a, h1d, h2a, h2d;


  public VisionSystem(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();

    
    table = inst.getTable("datatable");
    h1a = table.getEntry("1.botangle");
    h2a = table.getEntry("2.botangle");
    h1d = table.getEntry("1.distance");
    h2d = table.getEntry("2.distance");
    //System.err.println("We fuqin out here");
    

  }
    

  @Override
  public void periodic(){
   try{
      hatch1.angle = h1a.getDouble(-100);
      hatch2.angle = h2a.getDouble(-100);
      hatch1.distance = h1d.getDouble(-100);
      hatch2.distance = h2d.getDouble(-100);
    
       updateSmartDashboard();
    }catch(Exception e){
      System.err.println(e);
    }
}

  

  private void updateSmartDashboard(){
     
        SmartDashboard.putStringArray("Hatches Visable", toStrings);
        SmartDashboard.putNumber("getRightHatchAngle()", hatch1.angle);
        SmartDashboard.putNumber("getLeftHatchAngle()", hatch2.angle);



    }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public double getRightHatchAngle(){
    if (!hatch1.isReal()){
      return hatch2.angle;
    } else if (!hatch2.isReal()){
      return hatch1.angle;
    } else {
    double angle = Math.max(hatch1.angle, hatch1.angle);
    return angle;
    }
  }

  public double getLeftHatchAngle(){
    if (!hatch1.isReal()){
      return hatch2.angle;
    } else if (!hatch2.isReal()){
      return hatch2.angle;
    } else {
    double angle = Math.max(hatch1.angle, hatch2.angle);
    return angle;
    }
  }

}
