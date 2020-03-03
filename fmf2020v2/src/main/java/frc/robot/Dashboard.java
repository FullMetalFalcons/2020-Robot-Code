/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

/**
 * Dashboard
 */
public  class Dashboard {

  public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  public static final ShuffleboardTab subsystemsTab = Shuffleboard.getTab("Subsystems");
  public static final ShuffleboardTab commandsTab = Shuffleboard.getTab("Commands");
  
  public static  NetworkTableEntry shooterRPM;
  public static  NetworkTableEntry MaxRPM;

  public static void  init()
  {
    
    Dashboard.MaxRPM =  subsystemsTab.add("Max  RPM", 1).getEntry(); 

    Dashboard.shooterRPM = subsystemsTab
        .add(" Shooter RPM", 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)) // specify widget properties here
        .getEntry();


  }

}