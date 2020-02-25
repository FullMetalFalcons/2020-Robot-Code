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

public class IntakeSystem extends CommandBase {
  /**
   * Creates a new IntakeSystem.
   */
  private DigitalInput limit;
  private double startTime;
  private boolean done;
  private final int waitTime = 3;

  public IntakeSystem() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.conveyer);
    limit = new DigitalInput(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    startTime = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limit.get()) {
      // Robot.conveyer.rollerIn();
      Timer.delay(1);
      Robot.conveyer.beltIn();
    }
    if (System.currentTimeMillis() - startTime > waitTime){
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (done = true) {
      Robot.conveyer.rollerStop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
