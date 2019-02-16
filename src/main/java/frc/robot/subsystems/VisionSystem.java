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

  public static final int MAX_HATCH_COUNT = 3;
  private HatchLocation[] hatches;
  NetworkTable table;
  String[] toStrings;
  Boolean inited = false;

private NetworkTableEntry identifiers;

  public VisionSystem(){
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    table = inst.getTable("datatable");
    try {
      identifiers = table.getEntry("identifiers");
    } catch (Exception e){
      //System.err.println(e);
    }

    hatches = new HatchLocation[5];
    toStrings = new String[5];
    for(int i = 0; i < MAX_HATCH_COUNT; i++){
      hatches[i] = new HatchLocation();


      
    }
    inited = true;
    

    

    
  }

  @Override
  public void periodic(){
  try{
   String[] identStr = identifiers.getStringArray(null);
   if (inited)
    for(int i = 0; i < MAX_HATCH_COUNT; i++){
      //hatches[i].distance = 
      hatches[i].distance = 1;
      table.getEntry(identStr[i] + ".distance").getDouble(-100);
      hatches[i].angle = table.getEntry(identStr[i] + ".angle").getDouble(-100);
      hatches[i].identifier = identStr[i];
      toStrings[i] = hatches[i].toString();
     }
    } catch (Exception e){
      //System.err.println(e);
    }
   updateSmartDashboard();
  }

  private void updateSmartDashboard(){
     
        SmartDashboard.putStringArray("Hatches Visable", toStrings);
      

    }

  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
