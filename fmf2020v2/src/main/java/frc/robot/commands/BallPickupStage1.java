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
import frc.robot.Robot;

public class BallPickupStage1 extends CommandBase {
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
    Robot.conveyer.rollerIn();
    Robot.conveyer.beltIn();
    }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    done = Robot.conveyer.isBallAvailable();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.conveyer.rollerStop();
    Robot.conveyer.beltStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
