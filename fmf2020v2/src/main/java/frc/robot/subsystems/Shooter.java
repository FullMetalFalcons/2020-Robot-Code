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
   private double maxRPM = 3000;

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

    SmartDashboard.putNumber("Shooter P Gain", kP);
    SmartDashboard.putNumber("Shooter I Gain", kI);
    SmartDashboard.putNumber("Shooter D Gain", kD);
    SmartDashboard.putNumber("Shooter I Zone", kIz);
    SmartDashboard.putNumber("Shooter Feed Forward", kFF);
    SmartDashboard.putNumber("Shooter Max Output", kMaxOutput);
    SmartDashboard.putNumber("Shooter Min Output", kMinOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void ShootOut() {
    // rightWheel.set(1);

    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    double setPoint = maxRPM;
    pidController.setReference(setPoint, ControlType.kVelocity);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", shooterEncoder.getVelocity());
  }

  public void ShootStop() {
    rightWheel.set(0);
  }
}
