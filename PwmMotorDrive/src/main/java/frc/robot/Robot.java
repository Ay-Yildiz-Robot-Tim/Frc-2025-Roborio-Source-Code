// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Serial;

import javax.lang.model.util.ElementScanner14;

import org.opencv.core.Point;
import org.w3c.dom.events.MouseEvent;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PlatformMovement;
import frc.robot.subsystems.RobotArmController;
import frc.robot.Constants.PwmChannelContants;
import frc.robot.subsystems.ElevatorController;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.subsystems.PneumaticSystem;
//import com.kauailabs.navx.frc.AHRS;roller();

class YC_Time {
static int myTimeStamp;
static int myRunTime;
static int isRunning;
}

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  
  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }
  
  //basınç sensörü kullanımı
  private AnalogInput pressureSensor;

  // joyistick tanımlamaları
  private Joystick joystick;
  private Joystick joystick1;
  //rasberryin göndereceği datayı tanımlayın
  private NetworkTable table;
  
  //movent hesaplama kütüphanesi tanımlaması
  private PlatformMovement platformMovent;

  //asansör motorunun tanımlanması
  ElevatorController elevatorMotor;

  //Kol Sınıfının Tanımlanması
  RobotArmController robotArm;

  //encoder tanımlamalaro
  private Encoder elvatorEncoder;
  private Encoder armEncoder;

  private Timer timer = new Timer();
  private Timer timer2 = new Timer();
  private Timer timer3 = new Timer();
  
  // piömatik class tanımlanması
  PneumaticSystem pneumatic;
  //setPoint@
  int setPoint = 2120;
  int armSetPoint = -498;
  int lastPointArm = -498;
  int lastPointElevator = 2120;
  int elevatorPulse;
  int armPulse;
  long count = 0;
  char pointChar = ' ';
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotInit() {
    //joyistic tanımlamasız
    joystick = new Joystick(0);  // Joystick 0. portta
    joystick1 = new Joystick(1);
    //movent classını projeye dahil etme
    platformMovent = new PlatformMovement();

    elvatorEncoder = new Encoder(0,1);
    armEncoder = new Encoder(2,3);

    elevatorMotor = new ElevatorController();
    
    robotArm = new RobotArmController();

    pressureSensor = new AnalogInput(PwmChannelContants.pressureChannel);

    pneumatic = new PneumaticSystem();
    // PID çıktısını -1 ile 1 arasında sınırlama

  }


  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    YC_Time.myTimeStamp = 0;
    YC_Time.myRunTime = 0;
    YC_Time.isRunning = 1;

    double A_R_Axis;
    double A_L_Axis;
    double A_X_Axis;
    double A_MotorSpeed[];
    double A_LeftSpeed;
    double A_RightSpeed;

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    timer2.reset();
    timer2.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  //encoder değerlerini çekme
  elevatorPulse = -elvatorEncoder.get() + lastPointElevator;
  armPulse = -armEncoder.get() + lastPointArm;
    if(timer2.get() <= 5){
      platformMovent.MoventManual(0, 1, 0);
    }
    else{
      platformMovent.MoventManual(0, 0, 0);
      setSetPointArm(180);
      elevatorMotor.SetMotorSpeed(setPoint, elevatorPulse);
      System.out.println(timer.get());
      if(timer3.get() >= 2.0){
        robotArm.SetMotorSpeed(armSetPoint, armPulse);
        timer.stop();
        System.out.println(armPulse + " " + elevatorPulse);

      }    
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    
    // this line or comment it out.
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


  //joyisctick verilerini oku
  double joystickBack = joystick.getRawAxis(2);
  double joystickFront= joystick.getRawAxis(3);
  double joyistickX = joystick.getRawAxis(4);

  //kol Verilerini Çekme
  int joyistckArm = joystick.getPOV();
  
  //valfi kontrol edecek verileri çekme
  boolean joyisctickValfOn = joystick.getRawButton(6);
  boolean joyisctickValfOff = joystick.getRawButton(5); 
  
  //Button Değerlerini Çekme Joystick
  boolean joystickButtonA = joystick.getRawButton(1);
  boolean joystickButtonB = joystick.getRawButton(2);
  boolean joystickButtonX = joystick.getRawButton(3);
  boolean joystickButtonY = joystick.getRawButton(4);

  //encoder değerlerini çekme
  elevatorPulse = -elvatorEncoder.get() + lastPointElevator;
  armPulse = -armEncoder.get() + lastPointArm;

  //istenen yükseklik değerine ulaşma
  int point = joystickButtonA ? 100 : joystickButtonB ? 1000 : joystickButtonX ? 2000 : joystickButtonY ? 3000: 0;
  
  //setpoint değerini güncelleme
  setSetPoint(point);
  setSetPointArm(joyistckArm);

  pneumatic.SystemFrover(joyisctickValfOn, joyisctickValfOff);
  platformMovent.MoventManual(joystickBack, joystickFront, joyistickX);
  elevatorMotor.SetMotorSpeed(setPoint, elevatorPulse);
  armLow(pointChar, joyisctickValfOff);
  if(timer.get() >= 2.0){
    robotArm.SetMotorSpeed(armSetPoint, armPulse);
    timer.stop();
  }
  System.out.println(pointChar + " " + armSetPoint);
}

  //kumdandan gelicek joyistick verisini kalıcı olarak atama
  private void setSetPoint(int point){
    if(point == 100){      
      armSetPoint = -1425;
      setPoint = 2200; 
    }
    else if(point == 1000){
      setPoint = 2450;
      armSetPoint = -1025;
      pointChar = 'b';
    }
    else if(point == 2000){
      setPoint = 4240;
      armSetPoint = -1035;
      pointChar = 'c';
    }
    else if(point == 3000){
      setPoint = 4100;
      armSetPoint = -850;
      pointChar = 'y';
    }
  }
  
  private void setSetPointArm(int point){
    if(point == 180){
      armSetPoint = -1400;
      setPoint = 3300;
      timer.reset();
      timer.start(); // Timer başlatılıyor
      timer3.start();
    }

    else if(point == 0){
      setPoint = 200;
    }

    else if(point == 90){
      armSetPoint = -885;
      setPoint = 4000;
    }

    else if(point == 270){
      armSetPoint = -835;
      setPoint = 3807;
    }
  }

  private void armLow(char x, boolean joyisctickValfOff){
    if(joyisctickValfOff){
      if(x == 'b' || x == 'c'){
        armSetPoint = -910;
        pointChar = ' ';
      }
    }
  }
@Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
