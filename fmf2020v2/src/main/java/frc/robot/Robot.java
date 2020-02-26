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
import frc.robot.commands.IntakeSystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.WheelOfFortune;

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
   private DigitalInput limit;

   public static IntakeSystem intakeSystem;

   private RobotContainer robotContainer;
   private edu.wpi.first.wpilibj2.command.Command autonomousCommand;

   private NetworkTable table;

   private NetworkTableEntry targetX;
   private NetworkTableEntry targetY;

   public static IntakeSystem autoIntake;

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
    limit = new DigitalInput(0);

    robotContainer = new RobotContainer();


    table=NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("DriverCam");

    targetX=table.getEntry("yaw");
    targetY=table.getEntry("pitch");
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

    if (psController.getRawButton(1)){
      intake.intakeConstant();
      intake.intakeExtend();
      conveyer.rollerIn();
    } else {
      intake.intakeStop();
      intake.intakeIn();
      conveyer.rollerStop();
    }

    if (psController.getRawButton(11)){
      conveyer.beltIn();
    } else {
      conveyer.beltStop();
    }

    if (psController.getRawButton(12)){
      conveyer.beltOut();
    } else {
      conveyer.beltStop();
    }

    if (psController.getRawButton(3)){
      climber.downWeGo();
    } else {
      climber.pleaseStop();
    }

    if (psController.getRawButton(4)){
      conveyer.rollerIn();
    } else {
      conveyer.rollerStop();
    }

    if (psController.getRawButton(5)){
      conveyer.liftUp();
    }

    if (psController.getRawButton(6)){
      conveyer.liftDown();
    }

    if (psController.getRawButton(7)){
      climber.elevatorDown();
    } else {
      climber.elevatorStop();
    }

    if (psController.getRawButton(8)){
      climber.elevatorUp();
    } else {
      climber.elevatorStop();
    }

    rotationAjust = 0;
    distanceAdjust = 0;

    if (psController.getRawButton(12)){
      
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
    
    if (limit.get()) {
      
    }

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
