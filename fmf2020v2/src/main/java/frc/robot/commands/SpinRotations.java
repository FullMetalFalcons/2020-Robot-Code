/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelOfFortune;

public class SpinRotations extends CommandBase {
  /**
   * Creates a new SpinRotations.
   */

   private final WheelOfFortune wheelOfFortune;

   private String initialColor;
   private int colorCount;
   private boolean wasLastInitial;


  public SpinRotations(WheelOfFortune wheelOfFortune) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.wheelOfFortune = wheelOfFortune;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    initialColor = wheelOfFortune.getColor();
    colorCount = 0;
    wasLastInitial = true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    wheelOfFortune.spinWheel();
    boolean isCurrentInitial = wheelOfFortune.getColor().equals(initialColor);
    wheelOfFortune.getColor();
    if (!wasLastInitial && isCurrentInitial){
      colorCount++;
    }
    wasLastInitial = isCurrentInitial;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wheelOfFortune.stopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (colorCount >=6) && !wheelOfFortune.getColor().equals(initialColor);
  }
}
