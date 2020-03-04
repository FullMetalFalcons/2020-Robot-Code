
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

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Dashboard;

public class Shooter extends SubsystemBase {
  /**
   * Creates a new Shooter.
   */

   private CANSparkMax rightWheel;
   private CANSparkMax leftWheel;

   
   private int shooterRPM; 
  //  private CANPIDController pidController;

   private CANEncoder shooterEncoder;

  private double kP = 0.00025;
   private double kI = 0.000001;
   private double kD = 0.0000;
   private double kIz = 0;
   private double kFF = 0.000175;  
   private double kMaxOutput = 1;
   private double kMinOutput = 0;
   private double maxRPM = 5700;

   /* 
   5500rpm
   P:0.000250
   I:0.000001
   D:0.0
   FF: 0.000175

   5000rpm
   P:0.000250
   I:0.0000010
   D:0.0
   FF:0.0000150

   4500rpm
   P:
   I:
   D:
   FF:
   */
   
   private CANPIDController pidController;

  public Shooter() {

    rightWheel = new CANSparkMax(15, MotorType.kBrushless);
    leftWheel = new CANSparkMax(16, MotorType.kBrushless);

    shooterEncoder = rightWheel.getEncoder();

    rightWheel.restoreFactoryDefaults();
    leftWheel.restoreFactoryDefaults();
  
    leftWheel.follow(rightWheel, true);

    // rightWheel.setSmartCurrentLimit(40);
    // leftWheel.setSmartCurrentLimit(40);


    pidController = rightWheel.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    Dashboard.shooterRPM.addListener( (entry) -> {
      shooterRPM= (int) Math.round(maxRPM * entry.value.getDouble());
   }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
   Dashboard.MaxRPM.addListener( (entry) -> {
    maxRPM=  entry.value.getDouble();
   }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    
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
    SmartDashboard.putNumber("RPM", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("Target RPM", shooterRPM);
    
  }

  public void ShooterShootByRPM()
  {
    double p = SmartDashboard.getNumber("Shooter P Gain", 0);
    double i = SmartDashboard.getNumber("Shooter I Gain", 0);
    double d = SmartDashboard.getNumber("Shooter D Gain", 0);
    double iz = SmartDashboard.getNumber("Shooter I Zone", 0);
    double ff = SmartDashboard.getNumber("Shooter Feed Forward", 0);
    double max = SmartDashboard.getNumber("Shooter Max Output", 0);
    double min = SmartDashboard.getNumber("Shooter Min Output", 0);

    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }


    pidController.setReference(shooterRPM, ControlType.kVelocity);
  }

  public void ShootOut() {
    rightWheel.set(.95);

        // double setPoint = maxRPM;
    //pidController.setReference(setPoint, ControlType.kVelocity);

    // SmartDashboard.putNumber("SetPoint", setPoint);
  }

  public void shootLow(){
    pidController.setReference(2000, ControlType.kVelocity);
  }

  public void ShootStop() {
    rightWheel.set(0);
  }
}
