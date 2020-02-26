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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /**
   * Creates a new Climber.
   */

   private CANSparkMax elevatorMotor;
   private CANSparkMax winchMotor;

  //  private CANPIDController pidController;
  //  private CANEncoder encoder;

  //  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

   private  final AnalogInput pressureSensor;

  public Climber() {
    elevatorMotor = new CANSparkMax(13, MotorType.kBrushless);
    winchMotor = new CANSparkMax(14, MotorType.kBrushless);

    elevatorMotor.restoreFactoryDefaults();
    winchMotor.restoreFactoryDefaults();

    winchMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setIdleMode(IdleMode.kBrake);


    // pidController = elevatorMotor.getPIDController();

    // encoder = elevatorMotor.getEncoder();

    pressureSensor = new AnalogInput(2);

    //CHANGE THESE NUMBERS YOU FOOL
    // kP = 0.1;
    // kI = 0.0001;
    // kD = 1;
    // kIz = 0;
    // kFF = 0;
    // kMaxOutput = 1;
    // kMinOutput = -1;

    // pidController.setP(kP);
    // pidController.setI(kI);
    // pidController.setD(kD);
    // pidController.setIZone(kIz);
    // pidController.setFF(kFF);
    // pidController.setOutputRange(kMinOutput, kMaxOutput);

    // SmartDashboard.putNumber("P Gain", kP);
    // SmartDashboard.putNumber("I Gain", kI);
    // SmartDashboard.putNumber("D Gain", kD);
    // SmartDashboard.putNumber("I Zone", kIz);
    // SmartDashboard.putNumber("Feed Forward", kFF);
    // SmartDashboard.putNumber("Max Output", kMaxOutput);
    // SmartDashboard.putNumber("Min Output", kMinOutput);
    // SmartDashboard.putNumber("Set Rotations", 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // double p = SmartDashboard.getNumber("P Gain", 0);
    // double i = SmartDashboard.getNumber("I Gain", 0);
    // double d = SmartDashboard.getNumber("D Gain", 0);
    // double iz = SmartDashboard.getNumber("I Zone", 0);
    // double ff = SmartDashboard.getNumber("Feed Forward", 0);
    // double max = SmartDashboard.getNumber("Max Output", 0);
    // double min = SmartDashboard.getNumber("Min Output", 0);
    // double rotations = SmartDashboard.getNumber("Set Rotations", 0);

    // if((p != kP)) { pidController.setP(p); kP = p; }
    // if((i != kI)) { pidController.setI(i); kI = i; }
    // if((d != kD)) { pidController.setD(d); kD = d; }
    // if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   pidController.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 

    // pidController.setReference(rotations, ControlType.kPosition);
    
    // SmartDashboard.putNumber("SetPoint", rotations);
    // SmartDashboard.putNumber("ProcessVariable", encoder.getPosition());
    // }

    SmartDashboard.putNumber("Pressure", this.getPressure());
  }

  public void elevatorUp() {
    elevatorMotor.set(1.0);
  }

  public void elevatorDown() {
    elevatorMotor.set(-1.0);
  }

  public void elevatorStop() {
    elevatorMotor.stopMotor();
  }

  public void upWeGo() {
    winchMotor.set(-1.0);
  }

  public void downWeGo() {
    winchMotor.set(1.0);
  }

  public void pleaseStop() {
    winchMotor.stopMotor();
  }

  public double getPressure(){
    // Output: 0.5V–4.5V linear voltage output. 0 psi outputs 0.5V, 100 psi outputs 2.5V, 200 psi outputs 4.5V
    return 200 * (pressureSensor.getVoltage() - 0.5) / 4.0;
  }
 
}
