
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /**
   * Creates a new ConveyerBelt.
   */

  private WPI_TalonSRX beltBottom;
  private WPI_TalonSRX beltTop;

  private DoubleSolenoid lift;

  public DigitalInput limitSwitch;

  public DutyCycleEncoder encoder;

  public Indexer() {
    beltBottom = new WPI_TalonSRX(5);
    beltTop = new WPI_TalonSRX(23);

    limitSwitch = new DigitalInput(0);

    lift = new DoubleSolenoid(6,7);

    encoder = new DutyCycleEncoder(0);

    beltBottom.follow(beltTop);

    beltTop.setInverted(false);

    beltTop.setNeutralMode(NeutralMode.Brake);
    beltBottom.setNeutralMode(NeutralMode.Brake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void beltUp() {
    beltTop.set(ControlMode.PercentOutput, -1);
  }

  public void beltFastOut() {
    beltTop.set(ControlMode.PercentOutput, -1);
  }

  public void beltDown() {
    beltTop.set(ControlMode.PercentOutput, 0.20);
  }

  public void beltStop() {
    beltTop.set(ControlMode.PercentOutput, 0);
  }

  public void conveyerUp(){
    lift.set(Value.kForward);
  }

  public void conveyerDown(){
    lift.set(Value.kReverse);
  }

  public double encoderRotations(){
    return encoder.getDistance();
  }

  public boolean isBallAvailable() {
    return !limitSwitch.get();
  } 
}
