// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Test Template for Robot Java Code
// Don't forget to add all imports for 3rd party libraries

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TalonFX motor = new TalonFX(0); // creates a new TalonFX with ID 0
  private final RelativeEncoder m_testEncoder = m_testSpark.getEncoder();
  private final XboxController m_xbox = new XboxController(2);
  private final PIDController m_pid = new PIDController(0.005, 0.00005, 0);
  private double currPos = 0;
  private double speed;
  private GenericEntry kP;
  private GenericEntry kI;
  private GenericEntry kD;
  double p;
  double i;
  double d;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_pid.reset();
    m_testEncoder.setPosition(0);
    m_pid.setSetpoint(75);

    kP = Shuffleboard.getTab("SmartDashboard").add("kP", 0.005).withWidget("Text View").getEntry();
    kI = Shuffleboard.getTab("SmartDashboard").add("kI", 0.00005).withWidget("Text View").getEntry();
    kD = Shuffleboard.getTab("SmartDashboard").add("kD", 0).withWidget("Text View").getEntry();

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic(){
    // SmartDashboard
    double rotationSpeed;
    currPos = m_testEncoder.getPosition();
    speed = m_pid.calculate(currPos, 75);
    m_pid.setP(kP.getDouble(0.005));
    m_pid.setI(kI.getDouble(0.0005));
    m_pid.setD(kD.getDouble(0));

    // Test 1: Motor will turn by xbox control. Observe smartDashboard if encoder values change and display
    // on dashboard.
    if (m_xbox.getBButton()){     // Test 2: Control motor by set point and WPI PID Loop
      motor.set(TalonFXControlMode.PercentOutput, speed); // runs the motor at wanted power
    }
    else{
      m_testSpark.set(0);
    }
    if (m_xbox.getYButton()){
      m_testEncoder.setPosition(0);
      m_pid.setSetpoint(75);
    }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

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
