/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

   private CANSparkMax rightWheel;
   private CANSparkMax leftWheel;

  public Shooter() {

    rightWheel = new CANSparkMax(15, MotorType.kBrushless);
    leftWheel = new CANSparkMax(16, MotorType.kBrushless);

    // leftWheel.follow(rightWheel);

    leftWheel.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ShootOut() {
    rightWheel.set(1);
    leftWheel.set(1);
  }

  public void ShootIn() {
    rightWheel.set(-0.7);
  }

  public void ShootStop() {
    rightWheel.set(0);
    leftWheel.set(0);
  }
}
