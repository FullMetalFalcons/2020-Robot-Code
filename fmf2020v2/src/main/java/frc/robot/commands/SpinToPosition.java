/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Collections;
import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WheelOfFortune;

public class SpinToPosition extends CommandBase {
  /**
   * Creates a new SpinToPosition.
   */

  private SuppliedValueWidget<Boolean> assignedColorWidget;
  private SuppliedValueWidget<Boolean> targetColorWidget;
  
  private static final Map<String, String> targetColorMap = Collections.unmodifiableMap(Map.of(
      "G", "Yellow",
      "B", "Red",
      "Y", "Green",
      "R", "Blue"
  ));

  private static final Map<String, String> colorNameMap = Collections.unmodifiableMap(Map.of(
      "G", "Green",
      "B", "Blue",
      "Y", "Yellow",
      "R", "Red"
  ));

  private final WheelOfFortune wheelOfFortune;
  private String targetColor;

  public SpinToPosition(WheelOfFortune wheelOfFortune) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.wheelOfFortune = wheelOfFortune;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var gameData = DriverStation.getInstance().getGameSpecificMessage();
    targetColor = targetColorMap.get(gameData);
    if (targetColorWidget != null && assignedColorWidget != null) {
      targetColorWidget.withProperties(Map.of("colorWhenTrue", targetColor));
      assignedColorWidget.withProperties(Map.of("colorWhenTrue", colorNameMap.get(gameData)));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wheelOfFortune.spinForColor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      wheelOfFortune.stopWheel();
    } else {
      wheelOfFortune.stopOnPos();
    }
    // wheelOfFortune.stopWheel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if  (targetColor == null){
      return true;
    }
    return targetColor == wheelOfFortune.getColor();
  }
}
