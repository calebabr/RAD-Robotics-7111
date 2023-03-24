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
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

// import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  TalonFX motor = new TalonFX(0); // creates a new TalonFX with ID 0
  CANSparkMax sparkMotor = new CANSparkMax(6, MotorType.kBrushless);
  // private final RelativeEncoder m_testEncoder = m_testSpark.getEncoder();
  private final XboxController m_xbox = new XboxController(2);
  private final PIDController falconPID = new PIDController(0,0,0);
  private final PIDController sparkPID = new PIDController(0,0,0);
  private double currFalconPos = 0;
  private double falconSpeed;
  private double currSparkPos = 0;
  private double sparkSpeed;
  private GenericEntry falconkP;
  private GenericEntry falconkI;
  private GenericEntry falconkD;
  private GenericEntry sparkkP;
  private GenericEntry sparkkI;
  private GenericEntry sparkkD;
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

    falconkP = Shuffleboard.getTab("SmartDashboard").add("falcon kP", 0.005).withWidget("Text View").getEntry();
    falconkI = Shuffleboard.getTab("SmartDashboard").add("falcon kI", 0.00005).withWidget("Text View").getEntry();
    falconkD = Shuffleboard.getTab("SmartDashboard").add("falcon kD", 0).withWidget("Text View").getEntry();
    sparkkP = Shuffleboard.getTab("SmartDashboard").add("spark kP", 0.005).withWidget("Text View").getEntry();
    sparkkI = Shuffleboard.getTab("SmartDashboard").add("spark kI", 0.00005).withWidget("Text View").getEntry();
    sparkkD = Shuffleboard.getTab("SmartDashboard").add("spark kD", 0).withWidget("Text View").getEntry();

    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,0);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic(){
    // SmartDashboard
    currFalconPos = motor.getSelectedSensorPosition();
    falconSpeed = falconPID.calculate(currFalconPos, 100000);
    currSparkPos = sparkMotor.getEncoder().getPosition();
    sparkSpeed = sparkPID.calculate(currSparkPos, 500);
    falconPID.setP(falconkP.getDouble(0.005));
    falconPID.setI(falconkI.getDouble(0.0005));
    falconPID.setD(falconkD.getDouble(0));
    sparkPID.setP(sparkkP.getDouble(0.005));
    sparkPID.setI(sparkkI.getDouble(0.0005));
    sparkPID.setD(sparkkD.getDouble(0));

    // Test 1: Motor will turn by xbox control. Observe smartDashboard if encoder values change and display
    // on dashboard.
    if (m_xbox.getBButton()){     
      if (currFalconPos < 100000){
        motor.set(ControlMode.PercentOutput, falconSpeed);
      }
      else if (currFalconPos > 100000 && currSparkPos < 500){
        sparkMotor.set(sparkSpeed); // yuh
      }
    }
    else{
      motor.set(TalonFXControlMode.PercentOutput, 0);
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
