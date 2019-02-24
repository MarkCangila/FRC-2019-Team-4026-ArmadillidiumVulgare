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
  public HatchLocation hatch1;
  public HatchLocation hatch2;
  NetworkTable table;
  String[] toStrings;
  Boolean inited = false;
  NetworkTableEntry h1a, h1d, h2a, h2d;
  NetworkTableInstance inst;

  public VisionSystem(){
    inst = NetworkTableInstance.getDefault();
    
    inst.startDSClient();
    
    table = inst.getTable("datatable");
   
    //System.err.println("We fuqin out here");
    

  }
    

  @Override
  public void periodic(){
   try{
      hatch1.updateAngle((double)table.getEntry("1.botangle").getNumber(-100)); 
      hatch2.updateAngle((double)table.getEntry("2.botangle").getNumber(-100)); 
     // hatch1.distance = table.getNumber("1.distance", -100);
     // hatch2.distance = table.getNumber("2.distance", -100);
    
       updateSmartDashboard();
    }catch(Exception e){
      //System.err.println(e);
    }
    SmartDashboard.putBoolean("connectedToTable", inst.isConnected());
    
}

  

  private void updateSmartDashboard(){
     
        SmartDashboard.putStringArray("Hatches Visable", toStrings);
        SmartDashboard.putNumber("getRightHatchAngle()", hatch1.getAngle());
        SmartDashboard.putNumber("getLeftHatchAngle()", hatch2.getAngle());



    }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  

}
