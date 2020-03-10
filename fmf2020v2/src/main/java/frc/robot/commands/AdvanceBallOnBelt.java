/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class AdvanceBallOnBelt extends CommandBase {
  /**
   * Creates a new AutoIndex.
   */

  private double startTime;

  private double rotations;

  public AdvanceBallOnBelt() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.conveyer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();
    //Robot.conveyer.beltUp();
    //Robot.conveyer.rollerIn();
    Robot.conveyer.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.conveyer.beltFastOut();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.conveyer.beltStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return System.currentTimeMillis() - startTime > 2000;
    return Robot.conveyer.distanceTravel() > 3.5 ;  //inches
  }
}
