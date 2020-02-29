/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class BallPickupStage1 extends InstantCommand {
  /**
   * Creates a new IntakeSystem.
   */
  private double startTime;
  private boolean done;
  private final int waitTime = 3;

  public BallPickupStage1() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.conveyer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    Robot.intake.intakeRetract();
    Robot.conveyer.beltUp();
    }
  // Called every time the scheduler runs while the command is scheduled.
  
}
