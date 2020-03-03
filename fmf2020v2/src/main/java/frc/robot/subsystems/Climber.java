/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

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

   private double kP = 0.1;
   private double kI = 0.0001;
   private double kD = 1;
   private double kIz = 0;
   private double kFF = 0;
   private double kMaxOutput = 1;
   private double kMinOutput = -1;

   private  final AnalogInput pressureSensor;

  public Climber() {
    elevatorMotor = new CANSparkMax(13, MotorType.kBrushless);
    winchMotor = new CANSparkMax(14, MotorType.kBrushless);

    elevatorMotor.restoreFactoryDefaults();
    winchMotor.restoreFactoryDefaults();

    winchMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setIdleMode(IdleMode.kBrake);


    pidController = elevatorMotor.getPIDController();

    encoder = elevatorMotor.getEncoder();

    pressureSensor = new AnalogInput(2);

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
    SmartDashboard.putNumber("Pressure", this.getPressure());
  }

  public void elevatorUp() {
    elevatorMotor.set(1.0);
  }

  public void elevatorDown() {
    elevatorMotor.set(-0.5);
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
    //return 200 * (pressureSensor.getVoltage() - 0.5) / 4.0;

    return 250 * pressureSensor.getVoltage() / 5.0 - 25.0;
  }

  public void elevatorTiltDown(){
    pidController.setReference(4, ControlType.kPosition);
  }

  public void elevatorTiltUp(){
    pidController.setReference(5, ControlType.kPosition);
  }

  public void elevatorLevel(){
    pidController.setReference(6, ControlType.kPosition);
  }
 
}
