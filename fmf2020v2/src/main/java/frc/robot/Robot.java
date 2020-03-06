/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//mike bloomberg 2

package frc.robot;

import com.fasterxml.jackson.databind.SequenceWriter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.CommandGroup;
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

   public XboxController driverController;
   public XboxController operatorController;

   public static Drivetrain drivetrain;
   public static WheelOfFortune wheelOfFortune;
   public static Shooter shooter;
   public static Climber climber;
   public static Indexer conveyer;
   public static Intake intake;

   private RobotContainer robotContainer;
   private edu.wpi.first.wpilibj2.command.Command autonomousCommand;

   private NetworkTable table;

   private NetworkTableEntry targetYaw;
   private NetworkTableEntry targetPitch;

   //public static IntakeSystem autoIntake;
   public static AdvanceBallOnBelt advanceBall;
   public static BallPickupStage1 pickupStage1;
   public static SequentialCommandGroup ballIntake;
   public static SequentialCommandGroup fourthBall;
   public static SequentialCommandGroup shootOneBall;

   private double rotationError;
   private double distanceError;

   private double angleTolerance=5;
   private double distanceTolerance=5;

   private double constantForce = 0.05;

   private double KpRot = -0.1;
   private double KpDist= -0.1;

   private double rotationAjust;
   private double distanceAdjust;
   public  Dashboard dashboard;

   //autos
   public SequentialCommandGroup shootandMoveAuto;

  @Override
  public void robotInit() {
    Dashboard.init();
    CameraServer.getInstance().startAutomaticCapture();

    driverController = new XboxController(0);
    operatorController = new XboxController(1);

    drivetrain = new Drivetrain();
    wheelOfFortune = new WheelOfFortune();
    shooter = new Shooter();
    climber = new Climber();
    conveyer = new Indexer();
    intake = new Intake();

    // autoIntake = new IntakeSystem();
    advanceBall = new AdvanceBallOnBelt();
    robotContainer = new RobotContainer();
    pickupStage1 = new BallPickupStage1();

    ballIntake = new SequentialCommandGroup(
      new BallPickupStage1(), 
      new WaitCommand(1), 
      new BallPickupStage2(),
      new WaitCommand(1), 
      new BallPickupStageX(),
      new WaitCommand(1),
      new AdvanceBallOnBelt()
      );
    
      fourthBall = new SequentialCommandGroup(
      new BallPickupStage1(), 
      new WaitCommand(1), 
      new BallPickupStage2(),
      new WaitCommand(1), 
      new BallPickupStageX()
      );

      shootOneBall = new SequentialCommandGroup(
        new InstantCommand(()->shooter.ShootByRPM(5500)),
        new WaitCommand(1),
        new AdvanceBallOnBelt(),
        new WaitCommand(0.5),
        new InstantCommand(()->shooter.ShootStop())
      );


    shootandMoveAuto = new SequentialCommandGroup
            (
            new InstantCommand(()->shooter.ShootByRPM(5500)),
            new WaitCommand(5),
            new InstantCommand(()->conveyer.beltFastOut()),
            new WaitCommand(5),  
            new InstantCommand(()->drivetrain.arcadeDrive(0.6, 0)),
            new WaitCommand(2),
            new InstantCommand(()->drivetrain.arcadeDrive(0, 0)),
            new InstantCommand(()->conveyer.beltStop()),
            new InstantCommand(()->shooter.ShootStop())
            );



    table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("VisionCamera");

    targetYaw = table.getEntry("targetYaw");
    targetPitch = table.getEntry("targetPitch");

    //conveyer.conveyerUp();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // robotContainer.resetOdometry();
    // autonomousCommand = robotContainer.getAutonomousCommand();

    
    shootandMoveAuto.schedule();
    // if (autonomousCommand != null) {
    //   autonomousCommand.schedule();
    // }

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    // Driver Controls

    double leftYStick = driverController.getY(Hand.kLeft);
    double rightYStick = driverController.getY(Hand.kRight);

    drivetrain.tankDrive(leftYStick, rightYStick);

    if (driverController.getRawButtonPressed(3)) {
      climber.upWeGo();
    }

    if (driverController.getRawButtonReleased(3)) {
      climber.pleaseStop();
    }

    if (driverController.getRawButtonPressed(4)) {
      climber.downWeGo();
    }

    if (driverController.getRawButtonReleased(4)) {
      climber.pleaseStop();
    }

    if (driverController.getRawButtonPressed(5)){
      drivetrain.tankDrive(0.5 * leftYStick,  0.5 * rightYStick);
    }

    if (driverController.getRawButtonReleased(5)){
      drivetrain.tankDrive(leftYStick, rightYStick);
    }


    rotationAjust = 0;
    distanceAdjust = 0;

    if (driverController.getRawButton(1)) {
      
      rotationError=targetYaw.getDouble(0.0);
      distanceError=targetPitch.getDouble(0.0);

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

      drivetrain.arcadeDrive(-distanceAdjust, rotationAjust);
    }

    //Operator Controls

    int dpad = operatorController.getPOV(0);

    if (dpad == 0){
      climber.elevatorTiltUp();
    } 
    if (dpad == 270) {
      climber.elevatorZero();
    } 
    if (dpad == 90) {
      climber.elevatorLevel();
    } 
    if (dpad == 180) {
      climber.elevatorTiltDown();
    }

    if (operatorController.getRawButton(1)){
      shootOneBall.schedule();
    }

    if (operatorController.getRawButtonPressed(2)) {
      intake.intakeIn();
      intake.intakeExtend();
    }

    if (operatorController.getRawButtonReleased(2)) {
      intake.intakeStop();
      intake.intakeRetract();
    }

    if (operatorController.getRawButton(3)) {
      fourthBall.schedule();
    }

    if (operatorController.getRawButton(4)){
      ballIntake.schedule();
    }

    if (operatorController.getRawButton(5)){
      conveyer.conveyerDown();
    }

    if (operatorController.getRawButton(6)){
      conveyer.conveyerUp();
    }

    if (operatorController.getRawButtonPressed(7)) {
      shooter.ShootStop();
    }

    if (operatorController.getRawButtonPressed(8)){
      shooter.ShooterShootByRPM();
    }

    if (operatorController.getRawButtonPressed(9)){
      climber.elevatorDown();
    }

    if (operatorController.getRawButtonReleased(9)){
      climber.elevatorStop();
    }

    if (operatorController.getRawButtonPressed(13)){
      conveyer.beltDown();
    }

    if (operatorController.getRawButtonReleased(13)){
      conveyer.beltStop();
    }

    if (operatorController.getRawButtonPressed(12)){
      conveyer.beltFastOut();
    } 

    if (operatorController.getRawButtonReleased(12)){
      conveyer.beltStop();
    }



  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
