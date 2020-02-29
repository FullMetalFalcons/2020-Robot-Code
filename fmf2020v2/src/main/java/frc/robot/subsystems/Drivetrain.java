/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  /**
   * Creates a new Drivetrain.
   */
  public final WPI_TalonSRX rightMaster = new WPI_TalonSRX(3);
  public final WPI_TalonSRX leftMaster = new WPI_TalonSRX(9);
  public final WPI_TalonSRX rightSlave = new WPI_TalonSRX(4);
  public final WPI_TalonSRX leftSlave = new WPI_TalonSRX(2);

  private DifferentialDrive diffDrive;

  private AHRS gyro;

  private final DifferentialDriveOdometry differentialDriveOdometry;
  private Pose2d savedPose;

  public static final double TRACK_WIDTH_METERS = 0.555625;
  public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
      TRACK_WIDTH_METERS);

  /** Voltage needed to overcome the motor’s static friction. kS */
  public static final double kS = 0.829;

  /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
  public static final double kV = 3.04;

  /** Voltage needed to induce a given acceleration in the motor shaft. kA */
  public static final double kA = 0.676;

  public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

  public Drivetrain() {

    diffDrive = new DifferentialDrive(leftMaster, rightMaster);
    addChild("DiffDrive", diffDrive);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(true);
    rightMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightSlave.setInverted(true);

    diffDrive.setSafetyEnabled(false);
    diffDrive.setExpiration(0.1);
    diffDrive.setMaxOutput(1.0);

    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    gyro = new AHRS(SPI.Port.kMXP);
    gyro.enableLogging(true);
    gyro.zeroYaw();

    zeroDriveTrainEncoders();

    setNeutralMode(NeutralMode.Coast);

    differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    TalonSRXConfiguration talonConfig = new TalonSRXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.neutralDeadband = 0.001;
    talonConfig.slot0.kF = 1023.0 / 6800.0;
    talonConfig.slot0.kP = 1.0;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = 0.0;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.openloopRamp = 0.25;
  }

  public void resetOdometry() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
    savedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(getHeading()));
    differentialDriveOdometry.resetPosition(savedPose, Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    differentialDriveOdometry.update(Rotation2d.fromDegrees(getHeading()), stepsToMeters(getLeftEncoderPosition()),
        stepsToMeters(getRightEncoderPosition()));
    SmartDashboard.putString("Pose", differentialDriveOdometry.getPoseMeters().toString());
    // This method will be called once per scheduler run

    System.out.println(gyro.getAngle());
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    diffDrive.tankDrive(leftSpeed, rightSpeed);
  }

  public void arcadeDrive(double speed, double rotation) {
    diffDrive.arcadeDrive(speed, rotation, false);
  }

  public void curvatureDrive(double speed, double rotation){
    diffDrive.curvatureDrive(speed, rotation, true);
  }


  private double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d;
  }

  public void zeroDriveTrainEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public int getRightEncoderPosition() {
    return rightMaster.getSelectedSensorPosition(0);
  }

  public int getLeftEncoderPosition() {
    return leftMaster.getSelectedSensorPosition(0);
  }

  public Pose2d getCurrentPose() {
    return differentialDriveOdometry.getPoseMeters();
  }

  public void saveCurrentPose() {
    savedPose = getCurrentPose();
  }

  public Pose2d getSavedPose() {
    return savedPose;
  }

  public void stop() {
    tankDriveVelocity(0, 0);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity()),
        stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity()));
  }

  public void setNeutralMode(NeutralMode neutralMode) {
    leftMaster.setNeutralMode(neutralMode);
    leftSlave.setNeutralMode(neutralMode);
    rightMaster.setNeutralMode(neutralMode);
    rightSlave.setNeutralMode(neutralMode);
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity())) / 20;
    var rightAccel = (rightVelocity - stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity())) / 20;

    var leftFeedForwardVolts = FEED_FORWARD.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = FEED_FORWARD.calculate(rightVelocity, rightAccel);

    leftMaster.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(leftVelocity), DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
    rightMaster.set(ControlMode.Velocity, metersPerSecToStepsPerDecisec(rightVelocity), DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);
  }

  public SequentialCommandGroup createCommandForTrajectory(Trajectory trajectory) {
    return new RamseteCommand(
            trajectory,
            this::getCurrentPose,
            new RamseteController(2, 0.7),
            DRIVE_KINEMATICS,
            this::tankDriveVelocity,
            this)
        .andThen(this::stop, this);
  }

  public static double insToRevs(double inches) {
    return inches / 4.5 * Math.PI;
  }

  public static double insPerSecToStepsPerDecisec(double inchesPerSec) {
    return insToSteps(inchesPerSec) * .1;
  }

  public static double insToSteps(double inches) {
    return (insToRevs(inches) * 4096);
  }

  public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

  public static double stepsToMeters(int steps) {
    return (Units.inchesToMeters(4.5) * Math.PI / 4096) * steps;
  }

  public static double metersToSteps(double meters) {
    return (meters / 0.48) * 4096;
  }

  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }
}