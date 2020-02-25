/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SpinRotations;
import frc.robot.commands.SpinToPosition;

import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain driveTrainSubsystem = new Drivetrain();



  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public static final double TRACK_WIDTH_METERS = 0.555625;
  public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
      TRACK_WIDTH_METERS);
  
      public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(2.09, 0.109, 0.172);
     
  public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = 
      new DifferentialDriveVoltageConstraint(FEED_FORWARD, DRIVE_KINEMATICS, 11);

  public RobotContainer() {
    // Configure the button bindings
    // configureButtonBindings();
  }

  // private void configureButtonBindings() {

  //   new JoystickButton(driverController, XboxController.Button.kX.value)
  //       .whenHeld(new SpinRotations(controlPanelSubsystem));

  //   new JoystickButton(driverController, XboxController.Button.kStart.value)
  //       .whenHeld(new SpinToPosition(controlPanelSubsystem));

  //   new JoystickButton(driverController, XboxController.Button.kBumperLeft.value)
  //      .whenPressed(driveTrainSubsystem::saveCurrentPose);

  //   new JoystickButton(driverController, XboxController.Button.kBumperRight.value).whenPressed(() ->
  //     driveTrainSubsystem.createCommandForTrajectory(
  //         TrajectoryGenerator.generateTrajectory(
  //           driveTrainSubsystem.getCurrentPose(),
  //           Collections.emptyList(),
  //           driveTrainSubsystem.getSavedPose(),
  //           new TrajectoryConfig(3, 2)
  //               .setKinematics(DRIVE_KINEMATICS)
  //               .addConstraint(VOLTAGE_CONSTRAINT)))
  //     .schedule());

  // }

  private void configureAutonomus(){
    try {
      // var startPose = new Pose2d(inchesToMeters(120), inchesToMeters(-95), Rotation2d.fromDegrees(0));
      // var endPose = new Pose2d(inchesToMeters(178), inchesToMeters(-36), Rotation2d.fromDegrees(0));
    }

    catch (Exception e) {
      DriverStation.reportError("Failed to load auto", true);
    }

    //try {
      // var trajectory = TrajectoryGenerator.generateTrajectory(
      //     new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
      //     Collections.emptyList(),
      //     new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
      //     new TrajectoryConfig(TrajectoryConstants.MAX_SPEED_AUTO / 2, TrajectoryConstants.MAX_ACCELERATION_AUTO / 2)
      //         .setKinematics(DriveTrainConstants.DRIVE_KINEMATICS)
      //         .addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT));

    //   var autoCommandGroup =
    //       driveTrainSubsystem.createCommandForTrajectory(trajectory)
    //       .andThen(driveTrainSubsystem::stop, driveTrainSubsystem);
        
    //   autoChooser.setDefaultOption("Straight", autoCommandGroup);
    // } catch (Exception e) {
    //   DriverStation.reportError("Failed to load auto", true);
    //}    

  }

  protected static Trajectory loadTrajectory(String trajectoryName) throws IOException {
    return TrajectoryUtil.fromPathweaverJson(
        Filesystem.getDeployDirectory().toPath().resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
  }
  
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void resetOdometry() {
    new InstantCommand(driveTrainSubsystem::resetOdometry, driveTrainSubsystem).schedule();
  }
}
