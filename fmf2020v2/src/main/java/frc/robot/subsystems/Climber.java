/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.xml.stream.events.EndDocument;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

   private CANPIDController pidController;
   private CANEncoder encoder;

   private double kP = 0.01;
   private double kI = 0;
   private double kD = 0;
   private double kIz = 0;
   private double kFF = 0;
   private double kMaxOutput = 0.6;
   private double kMinOutput = 0;

   private  final AnalogInput pressureSensor;

   private double startPos;

  public Climber() {
    elevatorMotor = new CANSparkMax(13, MotorType.kBrushless);
    winchMotor = new CANSparkMax(14, MotorType.kBrushless);

    elevatorMotor.restoreFactoryDefaults();
    winchMotor.restoreFactoryDefaults();

    winchMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setIdleMode(IdleMode.kBrake);

    elevatorMotor.setSmartCurrentLimit(20);

    pidController = elevatorMotor.getPIDController();

    encoder = elevatorMotor.getEncoder();

    pressureSensor = new AnalogInput(2);

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    startPos = encoder.getPosition();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Pressure", this.getPressure());

    SmartDashboard.putNumber("Elevator Height", encoder.getPosition());

    //lowest 10.16
    //highest 265
  }

  public void elevatorUp() {
    elevatorMotor.set(0.8);
  }

  public void elevatorDown() {
    elevatorMotor.set(-0.7);
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
    winchMotor.set(0);
  }

  public double getPressure(){
    // Output: 0.5V–4.5V linear voltage output. 0 psi outputs 0.5V, 100 psi outputs 2.5V, 200 psi outputs 4.5V
    //return 200 * (pressureSensor.getVoltage() - 0.5) / 4.0;

    return 250 * pressureSensor.getVoltage() / 5.0 - 25.0;
  }

  public void getCurrentHeight(){
    encoder.getPosition();
  }

  public void elevatorTiltDown(){
    pidController.setReference(startPos + 230, ControlType.kPosition);
  }

  public void elevatorTiltUp(){

    pidController.setReference(startPos + 300, ControlType.kPosition);
  }

  public void elevatorLevel(){
    pidController.setReference(startPos + 260, ControlType.kPosition);
  }

  public void elevatorZero(){
    pidController.setReference(startPos, ControlType.kPosition);
  }
 
  public void elevatorUpBoost() {
    double currentPos = encoder.getPosition();
    pidController.setReference(currentPos + 30,  ControlType.kPosition);
  }

}
