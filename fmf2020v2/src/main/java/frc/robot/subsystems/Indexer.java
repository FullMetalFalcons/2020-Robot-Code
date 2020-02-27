
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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /**
   * Creates a new ConveyerBelt.
   */

  private WPI_TalonSRX beltBottom;
  private WPI_TalonSRX beltTop;
  private WPI_TalonSRX wheelThing;

  private DoubleSolenoid lift;

  public DigitalInput limitSwitch;

  public Indexer() {
    beltBottom = new WPI_TalonSRX(5);
    beltTop = new WPI_TalonSRX(23);
    wheelThing = new WPI_TalonSRX(7);

    limitSwitch = new DigitalInput(0);

    lift = new DoubleSolenoid(6,7);

    beltBottom.follow(beltTop);

    beltTop.setInverted(false);

    beltTop.setNeutralMode(NeutralMode.Brake);
    beltBottom.setNeutralMode(NeutralMode.Brake);

    wheelThing.configFactoryDefault();
    
    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.slot0.kP = 0.1;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    wheelThing.setNeutralMode(NeutralMode.Coast);

    wheelThing.configAllSettings(talonConfig);
    wheelThing.overrideLimitSwitchesEnable(false);
    wheelThing.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    wheelThing.setSensorPhase(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void beltIn() {
    beltTop.set(ControlMode.PercentOutput, 1);
  }

  public void beltOut() {
    beltTop.set(ControlMode.PercentOutput, -1);
  }

  public void beltStop() {
    beltTop.set(ControlMode.PercentOutput, 0);
  }

  public void rollerIn() {
    wheelThing.set(ControlMode.Position, 0.5 * 4096);
  }

  public void rollerStop() {
    wheelThing.set(ControlMode.PercentOutput, 0);
  }

  public void liftUp(){
    lift.set(Value.kForward);
  }

  public void liftDown(){
    lift.set(Value.kReverse);
  }

  public boolean isBallAvailable() {
    return !limitSwitch.get();
  }
  //Will add logic for sensor and indexing properly

  
}
