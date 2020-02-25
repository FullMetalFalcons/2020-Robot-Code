/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Dashboard
 */
public class Dashboard {

  public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
  public static final ShuffleboardTab subsystemsTab = Shuffleboard.getTab("Subsystems");
  public static final ShuffleboardTab commandsTab = Shuffleboard.getTab("Commands");

}