/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /**
   * Creates a new Intake.
   */

   private WPI_TalonSRX intakeMotor;

   private DoubleSolenoid intakeExtend;

  public Intake() {
    intakeMotor = new WPI_TalonSRX(12);

    intakeExtend = new DoubleSolenoid(5,4);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeExtend(){
    intakeExtend.set(Value.kReverse);
  }

  public void intakeIn(){
    intakeExtend.set(Value.kForward);
  }

  public void intakeFast(){
    intakeMotor.set(ControlMode.PercentOutput, 0.35);
  }

  public void intakeConstant(){
    intakeMotor.set(ControlMode.PercentOutput, 0.35);
  }

  public void intakeReverse(){
    intakeMotor.set(ControlMode.PercentOutput, -0.75);
  }

  public void intakeStop() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}
