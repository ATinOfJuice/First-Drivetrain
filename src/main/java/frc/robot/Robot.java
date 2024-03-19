// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.AnalogInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  //Motor IDs
  private static final int leftDeviceID1 = 2; 
  private static final int rightDeviceID1 = 4;
  private static final int leftDeviceID2 = 3; 
  private static final int rightDeviceID2 = 5;
  private static final int intakeDeviceID = 6;
  private static final int flywheelDeviceID = 7;

  //Drives and Motor Assignment
  private CANSparkMax leftDrive1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushed);
  private CANSparkMax leftDrive2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushed);
  private CANSparkMax rightDrive1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushed);
  private CANSparkMax rightDrive2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushed);
  private CANSparkMax intakeMotor = new CANSparkMax(intakeDeviceID, MotorType.kBrushed);
  private CANSparkMax flywheelMotor = new CANSparkMax(flywheelDeviceID, MotorType.kBrushless);
  private DifferentialDrive robotDrive1 = new DifferentialDrive(leftDrive1, rightDrive1);
  private DifferentialDrive robotDrive2 = new DifferentialDrive(leftDrive2, rightDrive2);

  //Encoders
  private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

  //Controllers
  private XboxController controller1 = new XboxController(0);
  private XboxController controller2 = new XboxController(1);

  //Sensors
  private AnalogInput noteProx = new AnalogInput(0);
  //private DigitalInput noteLimit = new DigitalInput(0);

  //Timers and Timer Resets/Delays
  private final Timer autoTimer = new Timer();
  private final Timer autoIntakeReversedTimer = new Timer();
  private final Timer intakeReversedTimer = new Timer();
  private final Timer intakeReversedTimerDelay = new Timer();
  private boolean intakeReversedTimerReset = false;
  private final Timer flywheelReleaseTimer = new Timer();
  private final Timer flywheelReleaseTimerDelay = new Timer();
  private double autoShooterStartTime = 0;
  private double autoIntakeStartTime = 0;
  
  //Status' and Button Delays
  private boolean intakestatus = false;
  private boolean lBumperDelay = false;
  private boolean xButtonDelay = false;
  private boolean rBumperDelay = false;
  private boolean reversedintakestatus = false;

  //TeleOp Half Speed
  private double halved = 1;

  //Autonomous Paths for SmartDashboard
  private static final String basicAutoLeft = "Left Basic";
  private static final String basicAutoRight = "Right Basic";
  private static final String closeAuto = "Four Note Close";
  private static final String farAutoTwoNote = "Two Note Far";
  private static final String farAutoOneNote = "One Note Far";
  private String autoSelected;
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  //Alliance Choosing for Smart Dashboard
  private static final String blueAlliance = "Blue Alliance";
  private static final String redAlliance = "Red Alliance";
  private String allianceSelected;
  private final SendableChooser<String> allianceChooser = new SendableChooser<>();

  //Autonomous Multipliers
  private int straightFrictionMult = 1;
  private int turningFrictionMult = 1;
  //private int wallMult = 1;
  private int allianceMult = 0;

  public Robot() {
    SendableRegistry.addChild(robotDrive1, leftDrive1);
    SendableRegistry.addChild(robotDrive1, leftDrive2);
    SendableRegistry.addChild(robotDrive2, rightDrive1);
    SendableRegistry.addChild(robotDrive2, rightDrive2);
    leftDrive1.restoreFactoryDefaults();
    rightDrive1.restoreFactoryDefaults();
  }

  //Distance from IR sensor. Used with intake.
  public double getDistance() {
    return (Math.pow(noteProx.getAverageVoltage(), -1.2045)) * 27.726;
  }

  //Autonomous Functions
  public void autoShooterSequence(){
    if (autoTimer.get() - autoShooterStartTime <= 2){
      flywheelMotor.set(1);
    } else if ((autoTimer.get() - autoShooterStartTime > 2) && (autoTimer.get() - autoShooterStartTime < 3)){
      flywheelMotor.set(1);
      intakeMotor.set(0.9);
    } else {
      flywheelMotor.stopMotor();
      intakeMotor.stopMotor();
    }
  }
  public void autoIntakeSequence(){
    if (getDistance() > 20){
      if (autoIntakeReversedTimer.get() < 0.04){
        intakeMotor.set(0.9);
      } else {
        intakeMotor.stopMotor();
      }
    } else if (autoTimer.get() - autoIntakeStartTime > 1.5){
      intakeMotor.stopMotor();
    } else {
      intakeMotor.set(-0.9);
      autoIntakeReversedTimer.restart();
    }
  }
  public void autoForwardSlow(){
    robotDrive1.tankDrive(-0.5*straightFrictionMult, -0.5*straightFrictionMult);
    robotDrive2.tankDrive(-0.5*straightFrictionMult, -0.5*straightFrictionMult);
  }
  public void autoBackwardSlow(){
    robotDrive1.tankDrive(0.5*straightFrictionMult, 0.5*straightFrictionMult);
    robotDrive2.tankDrive(0.5*straightFrictionMult, 0.5*straightFrictionMult);
  }
  public void autoForwardFast(){
    robotDrive1.tankDrive(-1*straightFrictionMult, -1*straightFrictionMult);
    robotDrive2.tankDrive(-1*straightFrictionMult, -1*straightFrictionMult);
  }
  public void autoBackwardFast(){
    robotDrive1.tankDrive(1*straightFrictionMult, 1*straightFrictionMult);
    robotDrive2.tankDrive(1*straightFrictionMult, 1*straightFrictionMult);
  }
  public void autoTurnLeft(){
    robotDrive1.tankDrive(0.7*turningFrictionMult*allianceMult, -0.7*turningFrictionMult*allianceMult);
    robotDrive2.tankDrive(0.7*turningFrictionMult*allianceMult, -0.7*turningFrictionMult*allianceMult);
  }
  public void autoTurnRight(){
    robotDrive1.tankDrive(-0.7*turningFrictionMult*allianceMult, 0.7*turningFrictionMult*allianceMult);
    robotDrive2.tankDrive(-0.7*turningFrictionMult*allianceMult, 0.7*turningFrictionMult*allianceMult);
  }
  public void autoStop(){
    robotDrive1.tankDrive(0, 0);
    robotDrive2.tankDrive(0, 0);
  }

  //Robot Initilization
  @Override
  public void robotInit() {
    //Inverting Drives
    rightDrive1.setInverted(true);
    rightDrive2.setInverted(true);

    //TeleOp Displays
    SmartDashboard.putBoolean("Note Available: ", false);
    SmartDashboard.putNumber("Flywheel Speed: ", flywheelEncoder.getVelocity());
    SmartDashboard.putBoolean("Intake On: ", intakestatus);

    //Autonomous SmartDashboard Option Adding
    autoChooser.setDefaultOption("Left Basic", basicAutoLeft);
    autoChooser.setDefaultOption("Right Basic", basicAutoRight);
    autoChooser.addOption("Four Note Close", closeAuto);
    autoChooser.addOption("Two Note Far", farAutoTwoNote);
    autoChooser.addOption("One Note Far", farAutoOneNote);
    SmartDashboard.putData("Auto Choices", autoChooser);

    //Alliance SmartDashboard Option Adding
    allianceChooser.setDefaultOption("Red Alliance", redAlliance);
    allianceChooser.addOption("Blue Alliance", blueAlliance);
    SmartDashboard.putData("Alliance Choices", allianceChooser);

    //Camera Bootup
    CameraServer.startAutomaticCapture();
  }

  //Autonomous Initilization
  @Override
  public void autonomousInit() {
    //Selecting the Autonomous Pathing
    autoSelected = autoChooser.getSelected();
    System.out.println("Auto Selected: " + autoSelected);

    //Selecting the Alliance
    allianceSelected = allianceChooser.getSelected();
    System.out.println("Alliance selected: " + allianceSelected);
    if (allianceSelected == blueAlliance){
      allianceMult = -1;
    } else {
      allianceMult = 1;
    }

    //Restarting the Main Timer
    autoTimer.restart();
  }

  //Autonomous Periodic
  @Override
  public void autonomousPeriodic() {
    //Base Autonomous Movement for All Paths
    if (autoTimer.get() < 3.1){
      autoShooterSequence();
      autoStop();
    } else if (autoTimer.get() < 4){
      autoForwardSlow();
    } 
    if (autoTimer.get() > 4){
      switch (autoSelected){
        //Basic Auto Path
        case basicAutoLeft:
          if (autoTimer.get() < 5){
            autoForwardFast();
          } else if (autoTimer.get() < 5.4){
            autoBackwardFast();
          } else {
            autoStop();
          }
          break;
        case basicAutoRight:
          if (autoTimer.get() < 4.27){
            autoTurnLeft();
          } else if (autoTimer.get() < 7.3){
            autoForwardSlow();
          } else {
            autoStop();
          }
          break;
        //Auto for the Three Closest Notes
        case closeAuto:
          if (autoTimer.get() < 4.15){
            autoTurnLeft();
            autoIntakeStartTime = autoTimer.get();
          } else if (autoTimer.get() < 5.65){
            autoForwardSlow();
            autoIntakeSequence();
          } else if (autoTimer.get() < 7.65){
            autoBackwardSlow();
            autoIntakeSequence();
            autoShooterStartTime = autoTimer.get();
          } else if (autoTimer.get() < 7.8){
            autoTurnRight();
            autoShooterSequence();
          } else if (autoTimer.get() < 8){
            autoBackwardSlow();
            autoShooterSequence();
          } else if (autoTimer.get() < 10.7){
            autoShooterSequence();
            autoStop();
          } else {
            autoStop();
          }
          break;
        //Auto for the Far Notes Closest to the Stage
        case farAutoTwoNote:
          if (autoTimer.get() < 4.3){
            autoTurnLeft();
          } else if (autoTimer.get() < 6.3){
            autoForwardFast();
          } else {
            autoStop();
          }
          break;
        //Auto for the Far Notes Furthest from the Stage
        case farAutoOneNote:
          if (autoTimer.get() < 6){
            autoForwardSlow();
          } else if (autoTimer.get() < 6.2){
            autoTurnRight();
          } else if (autoTimer.get() < 7.5){
            autoForwardFast();
          } else {
            autoStop();
          }
          break;
      }
    }
  }

  //TeleOp Initialization
  @Override
  public void teleopInit() {
    //Timer Reset
    flywheelReleaseTimer.restart();
    flywheelReleaseTimerDelay.restart();
    intakeReversedTimer.restart();
    intakeReversedTimerDelay.restart();
    SmartDashboard.putBoolean("Note Available: ", false);
    SmartDashboard.putNumber("Flywheel Speed: ", flywheelEncoder.getVelocity());
    SmartDashboard.putBoolean("Intake On: ", intakestatus);
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    //Movement Controls
    if ((controller1.getLeftY() >= 0.1 || controller1.getLeftY() <= -0.1) && (controller1.getRightX() <= 0.1 && controller1.getRightX() >= -0.1)){
      robotDrive1.tankDrive(controller1.getLeftY()*halved, controller1.getLeftY()*halved);
      robotDrive2.tankDrive(controller1.getLeftY()*halved, controller1.getLeftY()*halved);
    } else if ((controller1.getLeftY() >= 0.1 || controller1.getLeftY() <= -0.1) && (controller1.getRightX() > 0.1 || controller1.getRightX() < -0.1)){
      if (halved == 0.5){
        robotDrive1.tankDrive(controller1.getLeftY()*0.5 - controller1.getRightX()*0.3, controller1.getLeftY()*0.5 + controller1.getRightX()*0.3);
        robotDrive2.tankDrive(controller1.getLeftY()*0.5 - controller1.getRightX()*0.3, controller1.getLeftY()*0.5 + controller1.getRightX()*0.3);
      } else {
        robotDrive1.tankDrive(controller1.getLeftY()*0.7 - controller1.getRightX()*0.3, controller1.getLeftY()*0.7 + controller1.getRightX()*0.3);
        robotDrive2.tankDrive(controller1.getLeftY()*0.7 - controller1.getRightX()*0.3, controller1.getLeftY()*0.7 + controller1.getRightX()*0.3);
      }
    } else {
      if (halved == 0.5){
        robotDrive1.tankDrive(-controller1.getRightX()*0.7, controller1.getRightX()*0.7);
        robotDrive2.tankDrive(-controller1.getRightX()*0.7, controller1.getRightX()*0.7);
      } else {
        robotDrive1.tankDrive(-controller1.getRightX(), controller1.getRightX());
        robotDrive2.tankDrive(-controller1.getRightX(), controller1.getRightX());
      }
    }
    if (controller1.getRightBumperPressed() && !rBumperDelay){
      if (halved == 0.5){
        halved = 1;
      } else {
        halved = 0.5;
      }
      rBumperDelay = true;
    }
    if (controller1.getRightBumperReleased()){
      rBumperDelay = false;
    }

    //Controller 1 & 2 Intake
    if (controller1.getLeftBumperPressed() || controller2.getAButtonPressed() && !lBumperDelay){
      intakestatus = !intakestatus;
      lBumperDelay = true;
    }
    if (controller1.getLeftBumperReleased() || controller2.getAButtonReleased()){
      lBumperDelay = false;
    }

    //Intake Auto-Stop
    if (getDistance() < 20){
      intakestatus = false;
      SmartDashboard.putBoolean("Note Available", true);
      System.out.println("Sensor Working");
      if (!intakeReversedTimerReset){
        intakeReversedTimer.restart();
        intakeReversedTimerReset = true;
      }
    } else {
      intakeReversedTimerReset = false;
    }

    //Shooting
    if (controller2.getBButton()){
      flywheelMotor.set(1);
      flywheelReleaseTimer.restart();
      SmartDashboard.putNumber("Flywheel Speed", flywheelEncoder.getVelocity());
    }
    if (!controller2.getBButton()){
      if (flywheelReleaseTimer.get() > 0.1 && flywheelReleaseTimer.get() < 2 && flywheelReleaseTimerDelay.get() > 2){
        flywheelMotor.set(1);
        intakestatus = true;
        SmartDashboard.putNumber("Flywheel Speed", flywheelEncoder.getVelocity());
        SmartDashboard.putBoolean("Note Available", false);
      } else if (flywheelReleaseTimer.get() < 0.1 || flywheelReleaseTimer.get() > 2){
        flywheelMotor.stopMotor();
      }
    }

    //Reverse intake
    if (controller2.getYButton()){
      reversedintakestatus = true;
      SmartDashboard.putBoolean("Note Available", false);
    } else if (intakeReversedTimer.get() > 0.1 && intakeReversedTimer.get() < 0.13 && intakeReversedTimerDelay.get() > 0.13){
      reversedintakestatus = true;
    } else {
      reversedintakestatus = false;
    }

    //Full Stop of Intake and Shooter
    if ((controller1.getLeftTriggerAxis() > 0.5 || controller2.getXButton()) && !xButtonDelay){
      flywheelMotor.stopMotor();
      intakestatus = false;
      xButtonDelay = true;
    }
    if (controller1.getLeftTriggerAxis() < 0.5 || !controller2.getXButton()){
      xButtonDelay = false;
    }

    //Intake setting
    if (intakestatus){
      intakeMotor.set(0.9);
      SmartDashboard.putBoolean("Intake On: ", intakestatus);
    } else if (reversedintakestatus){
      intakeMotor.set(-0.9);
    } else {
      intakeMotor.stopMotor();
      SmartDashboard.putBoolean("Intake On: ", intakestatus);
    }
  }
  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
