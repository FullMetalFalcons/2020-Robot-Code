
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

   private CANSparkMax rightWheel;
   private CANSparkMax leftWheel;

   private CANPIDController pidController;

   private CANEncoder shooterEncoder;

   private double kP = 0.1;
   private double kI = 0;
   private double kD = 0.01;
   private double kIz = 0;
   private double kFF = 0.000015;
   private double kMaxOutput = 1;
   private double kMinOutput = -1;

  public Shooter() {

    rightWheel = new CANSparkMax(15, MotorType.kBrushless);
    leftWheel = new CANSparkMax(16, MotorType.kBrushless);

    shooterEncoder = rightWheel.getEncoder();

    pidController = rightWheel.getPIDController();

    rightWheel.restoreFactoryDefaults();
    leftWheel.restoreFactoryDefaults();

    leftWheel.follow(rightWheel, true);

    rightWheel.setSmartCurrentLimit(40);
    leftWheel.setSmartCurrentLimit(40);

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ProcessVariable", shooterEncoder.getVelocity());
  }

  public void ShootOut() {
    // rightWheel.set(.95);
    pidController.setReference(5500, ControlType.kVelocity);
  }

  public void shootLow(){
    pidController.setReference(2000, ControlType.kVelocity);
  }

  public void ShootStop() {
    rightWheel.set(0);
  }
}
