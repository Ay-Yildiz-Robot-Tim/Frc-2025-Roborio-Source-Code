// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.Serial;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.PlatformMovement;


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
  }
  //motor sürücü pwm tanımlamaları
  private PWMVictorSPX leftFrontMotor;
  private PWMVictorSPX leftBackMotor;
  private PWMVictorSPX rightFrontMotor;
  private PWMVictorSPX  rightBackMotor;
  
  // joyistick tanımlamaları
  private Joystick joystick;

  //movent hesaplama kütüphanesi tanımlaması
  private PlatformMovement platformMovent;
  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotInit() {
    // Hareket motorlarına gidecek motor pinleri
    leftFrontMotor = new PWMVictorSPX(Constants.PwmChannelContants.leftFrontMotosPwmChannel);  // PWM port left front motor
    leftBackMotor = new PWMVictorSPX(Constants.PwmChannelContants.leftBackMotosPwmChannel);  // PWM port left back motor
    rightFrontMotor = new PWMVictorSPX(Constants.PwmChannelContants.rightFrontMotosPwmChannel);  // PWM port right front motor
    rightBackMotor = new PWMVictorSPX(Constants.PwmChannelContants.rightBackMotosPwmChannel);  // PWM port right back motor 

    //joyistic tanımlaması
    joystick = new Joystick(0);  // Joystick 0. portta
    
    //movent classını projeye dahil etme
    platformMovent = new PlatformMovement();
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

    //while(true)
    //{
      A_R_Axis = 0.2;
      A_L_Axis = 0;
      A_X_Axis = 0;
      A_MotorSpeed = platformMovent.PowerCalc(A_R_Axis, A_L_Axis, A_X_Axis);
      A_LeftSpeed = A_MotorSpeed[1];
      A_RightSpeed = A_MotorSpeed[0];
      leftBackMotor.set(A_LeftSpeed);
      leftFrontMotor.set(A_LeftSpeed);
      rightBackMotor.set(A_RightSpeed);
      rightFrontMotor.set(A_RightSpeed);
      Timer.delay(15);
      A_R_Axis = 0;
      A_L_Axis = 0;
      A_X_Axis = 0;
      A_MotorSpeed = platformMovent.PowerCalc(A_R_Axis, A_L_Axis, A_X_Axis);
      A_LeftSpeed = A_MotorSpeed[1];
      A_RightSpeed = A_MotorSpeed[0];
      leftBackMotor.set(A_LeftSpeed);
      leftFrontMotor.set(A_LeftSpeed);
      rightBackMotor.set(A_RightSpeed);
      rightFrontMotor.set(A_RightSpeed);
    //}
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  //joyisctic verilerini oku
  double joystickBack = joystick.getRawAxis(3);
  double joystickFront= joystick.getRawAxis(2);
  double joyistickX = joystick.getRawAxis(4);
  //System.out.println(joystickBack);
  //kütüphane verileri okuma
  double motorSpeed[] = platformMovent.PowerCalc(joystickFront, joystickBack, joyistickX);
  
  //kolay anlaşılması için değişkenlere atma
  double rightMotorSpeed = motorSpeed[0];
  double leftMotorSpeed = motorSpeed[1];
  
  //motorlara pwm ayarlama
  leftBackMotor.set(leftMotorSpeed);
  leftFrontMotor.set(leftMotorSpeed);
  rightBackMotor.set(rightMotorSpeed);
  rightFrontMotor.set(rightMotorSpeed);

  //System.out.println(Timer.getTimestamp());
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
