/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.util.Color;

public class WheelOfFortune extends SubsystemBase {
  /**
   * Creates a new WheelOfFortune.
   */

   private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

   private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
   private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
   private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
   private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
   private final Color kWhiteTarget = ColorMatch.makeColor(0.259, 0.488, 0.252);

   private final ColorMatch colorMatcher = new ColorMatch();

   private String colorString;

   private WPI_TalonSRX wofMotor = new WPI_TalonSRX(6);

   private static final double kS = 2.09;
   private static final double kV = 0.109;
   private static final double kA = 0.172;

   private static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

  public WheelOfFortune() {

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    talonConfig.slot0.kP = 0.1;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot1.kP = 1;
    talonConfig.slot1.kI = 0.0;
    talonConfig.slot1.kD = 0;
    wofMotor.setNeutralMode(NeutralMode.Brake);
    stopWheel();

    wofMotor.configAllSettings(talonConfig);
    wofMotor.overrideLimitSwitchesEnable(false);
    wofMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    wofMotor.setSensorPhase(true);

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
    colorMatcher.addColorMatch(kWhiteTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kWhiteTarget) {
      colorString = "White";
    } else if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "White";
    }
  }

  public String getColor(){
    return colorString;
  }

  public void spinWheel(){
    wofMotor.set(ControlMode.PercentOutput, 1);
  }

  public void spinForRotations() {
    spinSpeed(200 / 60);
  }

  public void spinForColor() {
    spinSpeed(180 / 60);
  }

  public void spinSpeed(double rpm) {
    var accel = (rpm - stepsPerDecisecToRPS(wofMotor.getSelectedSensorVelocity())) / .20;
    var leftFeedForwardVolts = FEED_FORWARD.calculate(rpm, accel);
    wofMotor.selectProfileSlot(0, 0);
    wofMotor.set(
        ControlMode.Velocity,
        rpmToStepsPerDecisec(180),
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
  }


  public void stopWheel(){
    wofMotor.set(ControlMode.PercentOutput, 0);
  }

  public void stopOnPos(){
    wofMotor.selectProfileSlot(1, 1);
    wofMotor.set(ControlMode.Position, wofMotor.getSelectedSensorPosition());
  }

  public static double stepsToRotations(int steps) {
    return steps / (double) 4096;
  }

  public static double stepsPerDecisecToRPS(int stepsPerDecisec) {
    return stepsToRotations(stepsPerDecisec) * 10;
  }

  public static double rotationsToSteps(double rotations) {
    return rotations * 4096;
  }

  public static double rpmToStepsPerDecisec(double rpm) {
    return rotationsToSteps(rpm) / 600.0;
  }
}





// .,.,.,., ,,.,.,.,.,..,.,.,., ,,.,.,.,.,.
// .,.,.,., ,,.,??????.777777., ,,.,.,.,.,.
// .,.,.,.,.,7.???????.7777777,Z,,.,.,.,.,.
// , ,., , 7777:??????,777777+OOOO, , , ,.,
// ,.,.,.:777777+?????,77777IOOOOOO~,.,., ,
// ,.,.,777777777?????,7777IOOOOOOOOZ.,., ,
// , ,.O?777777777????,7777OOOOOOOOZ,7,., ,
// .,.,OOOO.7777777???.777ZOOOOOO.7777.,.,.
// .,.OOOOOOOO.77777??.77ZOOOO,77777777,.,.
// .,.OOOOOOOOOOOI77:?.7:OO777777777777,.,.
// .,,OOOOOOOOOOOOO~7,..Z=7777777777777..,.
// ,.,.,.,.,..,.,.,.,.,,.,.,.,.,..,.,.,.,.,
// ,..7777777777777,?.,.$:7777777777777., ,
// ,.,77777777777???.$,$.$$777777777777., ,
// .,.77777777.?????$$.$$$$$$$,I7777777,.,.
// .,.,7777.???????$$$.$$ZZ$$$$$$,7777.,.,.
// .,.,7??????????$$$$.$$$$$$$$$$$$ZI7.,.,.
//  ....?????????$$$$$.$$$$Z$$$$$$$$$,.,.,.
// ,.,.,.,??????Z$$$$$,$$$$$$$$$$$$,,.,., ,
// ,.,.,.,.????7$$$$$$,$$$$$$$$$$$..,.,., ,
// ,.,.,.,.,.?.$$$$$$$,$$$$$$$.$..,.,.,., ,
// , ,.,.,.,..,.Z$$$$$,$$$$Z$,.,..,...,., ,
// .,.,.,., ,,.,.,.,.,..,.,.,., ,,.,.,.,.,.
// .,.,.,., ,,.,.,.,.,..,.,.,., ,,.,.,.,.,.