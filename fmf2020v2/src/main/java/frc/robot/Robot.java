/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WheelOfFortune;
import edu.wpi.first.wpilibj2.command.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

   public XboxController psController;

   public static Drivetrain drivetrain;
   public static WheelOfFortune wheelOfFortune;
   public static Shooter shooter;
   public static Climber climber;
   public static Indexer conveyer;
   public static Intake intake;

   //public static IntakeSystem intakeSystem;

   private RobotContainer robotContainer;
   private edu.wpi.first.wpilibj2.command.Command autonomousCommand;

   private NetworkTable table;

   private NetworkTableEntry targetX;
   private NetworkTableEntry targetY;

   //public static IntakeSystem autoIntake;
   public static AdvanceBallOnBelt advanceBall;
   public static BallPickupStage1 pickupStage1;
   public static SequentialCommandGroup ballIntake;
   private double rotationError;
   private double distanceError;

   private double angleTolerance=5;
   private double distanceTolerance=5;

   private double constantForce = 0.05;

   private double KpRot = -0.1;
   private double KpDist= -0.1;

   private double rotationAjust;
   private double distanceAdjust;

  @Override
  public void robotInit() {
    CameraServer.getInstance().startAutomaticCapture();

    psController = new XboxController(0);

    drivetrain = new Drivetrain();
    wheelOfFortune = new WheelOfFortune();
    shooter = new Shooter();
    climber = new Climber();
    conveyer = new Indexer();
    intake = new Intake();

    //autoIntake = new IntakeSystem();
    advanceBall = new AdvanceBallOnBelt();
    robotContainer = new RobotContainer();
    pickupStage1 = new BallPickupStage1();
    ballIntake = new SequentialCommandGroup(new BallPickupStage1(), new WaitCommand(1), new BallPickupStage2(), new WaitCommand(1), new BallPickupStage1(), new AdvanceBallOnBelt());
    

    table=NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("DriverCam");

    targetX=table.getEntry("yaw");
    targetY=table.getEntry("pitch");

    this.intake.intakeIn();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    robotContainer.resetOdometry();
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    double leftYStick = psController.getY(Hand.kLeft);
    double rightYStick = psController.getY(Hand.kRight);

    drivetrain.tankDrive(leftYStick, rightYStick);

    if (psController.getRawButton(11)){
      intake.intakeExtend();
    }

    if (psController.getRawButton(12)){
      intake.intakeIn();
    }

    if (psController.getRawButton(1)){
      ballIntake.schedule();
    }

    if (psController.getRawButtonPressed(2)){
      intake.intakeFast();
      intake.intakeExtend();
    }

    if (psController.getRawButtonReleased(2)){
      intake.intakeStop();
    }

    if (psController.getRawButtonPressed(3)){
      conveyer.beltFast();
    }

    if (psController.getRawButtonReleased(3)){
      conveyer.beltStop();
    }

    // if (psController.getRawButtonPressed(1)){
    //   intake.intakeConstant();
    // } 
    // if (psController.getRawButtonReleased(1)) {
    //   intake.intakeStop();
    // }

    // if (psController.getRawButtonPressed(2)){
    //   intake.intakeReverse();
    // } 
    // if (psController.getRawButtonReleased(2)){
    //   intake.intakeStop();
    // }

    // if (psController.getRawButtonPressed(11)){
    //   conveyer.beltIn();
    // } 
    // if (psController.getRawButtonReleased(11)) {
    //   conveyer.beltStop();
    // }

    // if (psController.getRawButtonPressed(12)){
    // //  conveyer.beltOut();
    // advanceBall.schedule();
    // }
    // //if (psController.getRawButtonReleased(12)) {
    // //  conveyer.beltStop();
    // //}

    // if (psController.getRawButton(5)){
    //   intake.intakeExtend();
    // }

    // if (psController.getRawButton(6)){
    //   intake.intakeIn();
    // }

    // if (psController.getRawButton(7)){
    //   climber.elevatorDown();
    // } 
    
    // if (psController.getRawButtonReleased(7)){
    //   climber.elevatorStop();
    // }

    // if (psController.getRawButton(8)){
    //   climber.elevatorUp();
    // } 

    // if (psController.getRawButtonReleased(8)){
    //   climber.elevatorStop();
    // }

    // if (psController.getRawButton(9)){
    //   ballIntake.schedule();
    // }

    // if (psController.getRawButtonPressed(3)){
    //   climber.upWeGo();
    // }

    // if (psController.getRawButtonReleased(3)){
    //   climber.pleaseStop();
    // }

    // if (psController.getRawButtonPressed(4)){
    //   climber.downWeGo();
    // }

    // if (psController.getRawButtonReleased(4)){
    //   climber.pleaseStop();
    // }

    // if (psController.getRawButtonPressed(14)){
    //   conveyer.beltFast();
    // }

    // if (psController.getRawButtonReleased(14)){
    //   conveyer.beltStop();
    // }
    

    rotationAjust = 0;
    distanceAdjust = 0;

    if (psController.getRawButton(13)){
      
      rotationError=targetX.getDouble(0.0);
      distanceError=targetY.getDouble(0.0);

      if(rotationError>angleTolerance)
        rotationAjust=KpRot*rotationError+constantForce;
      else
        if(rotationError<angleTolerance)
          rotationAjust=KpRot*rotationError-constantForce;

      if(distanceError>distanceTolerance)
        distanceAdjust=KpDist*distanceError+constantForce;
      else
        if(distanceError<distanceTolerance)
          distanceAdjust=KpDist*distanceError-constantForce;

      drivetrain.arcadeDrive(distanceAdjust, rotationAjust);
    }

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
