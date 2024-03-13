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
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private static final int leftDeviceID1 = 2; 
  private static final int rightDeviceID1 = 4;
  private static final int leftDeviceID2 = 3; 
  private static final int rightDeviceID2 = 5;
  private static final int intakeDeviceID = 6;
  private static final int flywheelDeviceID = 7;
  private CANSparkMax m_leftDrive1 = new CANSparkMax(leftDeviceID1, MotorType.kBrushed);
  private CANSparkMax m_leftDrive2 = new CANSparkMax(leftDeviceID2, MotorType.kBrushed);
  private CANSparkMax m_rightDrive1 = new CANSparkMax(rightDeviceID1, MotorType.kBrushed);
  private CANSparkMax m_rightDrive2 = new CANSparkMax(rightDeviceID2, MotorType.kBrushed);
  private CANSparkMax intakeMotor = new CANSparkMax(intakeDeviceID, MotorType.kBrushed);
  private CANSparkMax flywheelMotor = new CANSparkMax(flywheelDeviceID, MotorType.kBrushless);
  private DifferentialDrive m_robotDrive1 = new DifferentialDrive(m_leftDrive1, m_rightDrive1);
  private DifferentialDrive m_robotDrive2 = new DifferentialDrive(m_leftDrive2, m_rightDrive2);
  private XboxController controller1 = new XboxController(0);
  private XboxController controller2 = new XboxController(1);
  //private AnalogInput noteProx = new AnalogInput(0);
  //private DigitalInput noteLimit = new DigitalInput(0);
  private final Timer m_timer = new Timer();
  private final Timer flywheelReleaseTimer = new Timer();
  private final Timer flywheelReleaseTimerDelay = new Timer();

  private boolean intakestatus = false;
  private boolean flywheelstatus = false;
  private boolean aButtonDelay = false;
  private boolean bButtonDelay = false;
  private boolean xButtonDelay = false;
  private boolean rBumperDelay = false;
  private boolean previousBButtonPressed = false;

  private double halved = 1;

  private static final String basicAuto = "Basic";
  private static final String shootBasicAuto = "Shooting Only Basic";
  private static final String driveBasicAuto = "Driving Only Basic";
  private static final String closeAuto = "Four Note Close";
  private static final String farAutoTwoNote = "Two Note Far";
  private static final String farAutoOneNote = "One Note Far";
  private String autoSelected;
  private final SendableChooser<String> chooser = new SendableChooser<>();


  public Robot() {
    SendableRegistry.addChild(m_robotDrive1, m_leftDrive1);
    SendableRegistry.addChild(m_robotDrive1, m_leftDrive2);
    SendableRegistry.addChild(m_robotDrive2, m_rightDrive1);
    SendableRegistry.addChild(m_robotDrive2, m_rightDrive2);
    m_leftDrive1.restoreFactoryDefaults();
    m_rightDrive1.restoreFactoryDefaults();
  }

  /*public double getSensorDistance() {
    return (Math.pow(noteProx.getAverageVoltage(), -1.2045)) * 27.726;
  }*/

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightDrive1.setInverted(true);
    m_rightDrive2.setInverted(true);
    chooser.setDefaultOption("Basic", basicAuto);
    chooser.addOption("Shooting Only Basic", shootBasicAuto);
    chooser.addOption("Driving Only Basic", driveBasicAuto);
    chooser.addOption("Four Note Close", closeAuto);
    chooser.addOption("Two Note Far", farAutoTwoNote);
    chooser.addOption("One Note Far", farAutoOneNote);
    SmartDashboard.putData("Auto choices", chooser);
    CameraServer.startAutomaticCapture();
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    autoSelected = chooser.getSelected();
    System.out.println("Auto selected:" + autoSelected);
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (autoSelected){
      case basicAuto:
        
        break;
      case shootBasicAuto:

        break;
      case driveBasicAuto:

        break;
      case closeAuto:

        break;
      case farAutoTwoNote:

        break;
      case farAutoOneNote:

        break;
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    flywheelReleaseTimer.restart();
    flywheelReleaseTimerDelay.restart();
  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    if ((controller1.getLeftY() >= 0.1 || controller1.getLeftY() <= -0.1) && (controller1.getRightX() <= 0.1 && controller1.getRightX() >= -0.1)){
      m_robotDrive1.tankDrive(controller1.getLeftY()*halved, controller1.getLeftY()*halved);
      m_robotDrive2.tankDrive(controller1.getLeftY()*halved, controller1.getLeftY()*halved);
    } else if ((controller1.getLeftY() >= 0.1 || controller1.getLeftY() <= -0.1) && (controller1.getRightX() > 0.1 || controller1.getRightX() < -0.1)){
      if (halved == 0.5){
        m_robotDrive1.tankDrive(controller1.getLeftY()*0.5 - controller1.getRightX()*0.3, controller1.getLeftY()*0.5 + controller1.getRightX()*0.3);
        m_robotDrive2.tankDrive(controller1.getLeftY()*0.5 - controller1.getRightX()*0.3, controller1.getLeftY()*0.5 + controller1.getRightX()*0.3);
      } else {
        m_robotDrive1.tankDrive(controller1.getLeftY()*0.7 - controller1.getRightX()*0.3, controller1.getLeftY()*0.7 + controller1.getRightX()*0.3);
        m_robotDrive2.tankDrive(controller1.getLeftY()*0.7 - controller1.getRightX()*0.3, controller1.getLeftY()*0.7 + controller1.getRightX()*0.3);
      }
    } else {
      if (halved == 0.5){
        m_robotDrive1.tankDrive(-controller1.getRightX()*0.7, controller1.getRightX()*0.7);
        m_robotDrive2.tankDrive(-controller1.getRightX()*0.7, controller1.getRightX()*0.7);
      } else {
        m_robotDrive1.tankDrive(-controller1.getRightX(), controller1.getRightX());
        m_robotDrive2.tankDrive(-controller1.getRightX(), controller1.getRightX());
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
    if (controller1.getAButtonPressed() && !aButtonDelay){
      intakestatus = !intakestatus;
      aButtonDelay = true;
    }
    if (controller1.getAButtonReleased()){
      aButtonDelay = false;
    }
    //if (controller1.getAButtonPressed() && (getDistance() > 20)){
    if (controller1.getBButton()){
      flywheelMotor.set(1);
      flywheelReleaseTimer.restart();
    }
    if (!controller1.getBButton()){
      if (flywheelReleaseTimer.get() > 0.1 && flywheelReleaseTimer.get() < 2 && flywheelReleaseTimerDelay.get() > 2){
        flywheelMotor.set(1);
        intakestatus = true;
      } else if (flywheelReleaseTimer.get() < 0.1 || flywheelReleaseTimer.get() > 2){
        flywheelMotor.stopMotor();
      }
    }
    if (controller1.getAButtonPressed() && !aButtonDelay){
      intakestatus = !intakestatus;
      aButtonDelay = true;
    }
    if (controller1.getAButtonReleased()){
      aButtonDelay = false;
    }
    if ((controller1.getXButtonPressed() || controller1.getXButtonPressed()) && !xButtonDelay){
      flywheelstatus = false;
      intakestatus = false;
      xButtonDelay = true;
    }
    if (controller1.getXButtonReleased() || controller1.getXButtonReleased()){
      xButtonDelay = false;
    }
    if (intakestatus){
      intakeMotor.set(0.9);
    } else {
      intakeMotor.stopMotor();
    }
    /*if (noteLimit.get() && (flywheelReleaseTimer.get() < 0.1 || flywheelReleaseTimer.get() > 2)){
      intakestatus = false;
    }*/
    //Old Code for Flywheel and Feeding
    /*if (controller2.getAButtonPressed()){
      feederstatus = true;
      feederTimer.reset();
    }
    if (controller2.getBButtonPressed() && !bButtonDelay){
      flywheelstatus = !flywheelstatus;
      flywheelTimer.reset();
      bButtonDelay = true;
    }
    if (controller2.getBButtonReleased()){
      bButtonDelay = false;
    }*/
    /*if (controller1.getBButtonPressed()){
      flywheelstatus = true;
      previousBButtonPressed = true;
    }
    if (controller1.getBButtonReleased()){
      if (previousBButtonPressed == true){
        flywheelReleaseTimer.restart();
        previousBButtonPressed = false;
      }
    }
    if(flywheelReleaseTimerDelay.get() <= 3){
      flywheelReleaseTimer.restart();
    }
    if (flywheelReleaseTimer.get() > 0.1 && flywheelReleaseTimer.get() < 2 && controller1.getBButtonReleased()){
      flywheelstatus = true;
      intakestatus = true;
    } else if ((flywheelReleaseTimer.get() < 0.1 || flywheelReleaseTimer.get() > 2) && controller1.getBButtonPressed()){
      flywheelstatus = true;
      intakestatus = false;
    } else if ((flywheelReleaseTimer.get() < 0.1 || flywheelReleaseTimer.get() > 2) && controller1.getBButtonReleased()){
      flywheelstatus = false;
    }
    if ((controller1.getXButtonPressed() || controller1.getXButtonPressed()) && !xButtonDelay){
      flywheelstatus = false;
      intakestatus = false;
      xButtonDelay = true;
    }
    if (controller1.getXButtonReleased() || controller1.getXButtonReleased()){
      xButtonDelay = false;
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
    if (intakestatus){
      if (getDistance() > 20){
        intakeMotor.set(0.6);
      } else {
        intakeMotor.stopMotor();
        intakestatus = false;
      }
    }
    if (intakestatus){
      intakeMotor.set(0.9);
    } else {
      intakeMotor.stopMotor();
    }
    //Old code for Feeding
    if (feederstatus){
      if(feederTimer.get() < 3){
        intakeMotor.set(0.6);
      } else {
        feederstatus = false;
      }
    }
    if (flywheelstatus){
      flywheelMotor.set(1);
    } else {
      flywheelMotor.stopMotor();
    }
    if (getSensorDistance() < 20){
      System.err.println(getSensorDistance());
    }*/
  }
  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
