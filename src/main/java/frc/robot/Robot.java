// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.motorcontrol.*;
import com.revrobotics.REVLibError;

 

public class Robot extends TimedRobot {

  private DifferentialDrive m_roboDrive;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  Joystick joy1 = new Joystick(0); 
  Joystick joy2 = new Joystick(1);
  
  double j1y = 0;
  double j2y = 0;

  CANSparkMax FrontLeftMotor = new CANSparkMax(1, MotorType.kBrushless); 
  CANSparkMax FrontRightMotor = new CANSparkMax(2, MotorType.kBrushless); 
  CANSparkMax BackLeftMotor = new CANSparkMax(3, MotorType.kBrushless); 
  CANSparkMax BackRightMotor = new CANSparkMax(4, MotorType.kBrushless); 

  Double FrontLeftEncoder = 0.0;
  Double FrontRightEncoder = 0.0;
  Double BackLeftEncoder = 0.0;
  Double BackRightEncoder = 0.0;
  
  Double rightMotors; 
  Double leftMotors; 

  Double Speed; 

  private final MotorControllerGroup rightGroup = new MotorControllerGroup(FrontRightMotor, BackRightMotor); 
  private final MotorControllerGroup leftGroup = new MotorControllerGroup(FrontLeftMotor, BackLeftMotor); 

  private final DifferentialDrive m_robotDrive = new DifferentialDrive(FrontLeftMotor, FrontRightMotor);
  private final DifferentialDrive robotDrive = new DifferentialDrive(BackLeftMotor, BackRightMotor);
  //private final DifferentialDrive m_robotDrive = new DifferentialDrive(FrontLeftMotor, BackLeftMotor);
  //private final DifferentialDrive robotDrive = new DifferentialDrive(FrontRightMotor, BackRightMotor);  
  private final XboxController m_driverController = new XboxController(0);
  Joystick xboxcontoller = new Joystick(0);
  
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    BackLeftMotor.follow(FrontLeftMotor);  
    BackRightMotor.follow(FrontRightMotor);

    
    FrontLeftMotor.restoreFactoryDefaults();
    BackLeftMotor.restoreFactoryDefaults(); 
    FrontRightMotor.restoreFactoryDefaults();
    BackRightMotor.restoreFactoryDefaults();

    FrontLeftMotor.setInverted(false);
    FrontRightMotor.setInverted(false); 
    //BackRightMotor.setInverted(false); 
    //BackLeftMotor.setInverted(true); 

    //FrontLeftMotor.restoreFactoryDefaults();
    //BackLeftMotor.restoreFactoryDefaults(); 
    //FrontRightMotor.restoreFactoryDefaults();
    //BackRightMotor.restoreFactoryDefaults();

    FrontLeftMotor.setSmartCurrentLimit(80);
    FrontRightMotor.setSmartCurrentLimit(80); 
    BackLeftMotor.setSmartCurrentLimit(80); 
    BackRightMotor.setSmartCurrentLimit(80); 

    FrontLeftMotor.setOpenLoopRampRate(3);
    BackLeftMotor.setOpenLoopRampRate(3);
    FrontRightMotor.setOpenLoopRampRate(3);
    BackRightMotor.setOpenLoopRampRate(3);
    
    //FrontLeftMotor.setIdleMode(IdleMode.kBrake);
    //BackLeftMotor.setIdleMode(IdleMode.kBrake);
    //FrontRightMotor.setIdleMode(IdleMode.kBrake);
    //BackRightMotor.setIdleMode(IdleMode.kBrake);

    

                                                              // sets the left follower motors
    



   //leftMotors = new SpeedControllerGroup(FrontLeftMotor, BackLeftMotor);
   // rightMotors = new SpeedControllerGroup(FrontRightMotor, BackRightMotor);

  }

 
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    FrontLeftMotor.setIdleMode(IdleMode.kCoast); 
    FrontRightMotor.setIdleMode(IdleMode.kCoast); 
    BackLeftMotor.setIdleMode(IdleMode.kCoast); 
    BackRightMotor.setIdleMode(IdleMode.kCoast);
    
    //leftMotors = new SpeedControllerGroup(FrontLeftMotor, BackLeftMotor);
    //rightMotors = new SpeedControllerGroup(FrontRightMotor, BackRightMotor);

   //driver1 = new DifferentialDrive(FrontLeftMotor, FrontRightMotor);
  }

  @Override
  public void teleopPeriodic() {

     
     
    robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
    m_robotDrive.tankDrive(m_driverController.getLeftY(), m_driverController.getRightX()); 
     
    //m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), -m_driverController.getRightX());
    //robotDrive.arcadeDrive(m_driverController.getLeftY(), m_driverController.getRightX());
   



  }
  //change
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
