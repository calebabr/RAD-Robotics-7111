// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.controller.PIDController;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private XboxController m_xbox;

  private final VictorSPX m_frontLeft = new VictorSPX(1);
  private final VictorSPX m_backLeft = new VictorSPX(2);
  private final VictorSPX m_frontRight = new VictorSPX(3);
  private final VictorSPX m_backRight= new VictorSPX(4);

  private final PIDController m_pid = new PIDController(0.1, 0, 0);

  private double leftSpeed = 0;
  private double rightSpeed = 0;

  private double speed = 0;

  private AHRS ahrs;

  private float ahrsAngle;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    ahrs = new AHRS(Port.kMXP);
    ahrs.reset();

    m_pid.setSetpoint(0);
    m_pid.setTolerance(10);

    m_frontRight.setInverted(true);
    m_backRight.setInverted(true);
    
    m_rightStick = new Joystick(0);
    m_leftStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    ahrsAngle = ahrs.getPitch();
    if (m_xbox.getAButton()){
      speed = m_pid.calculate(ahrsAngle); //remap_range(ahrsAngle, -180, 180, -1, 1); // change this to PID Controller
      leftSpeed = speed;
      rightSpeed = speed;
    }
    else{
      leftSpeed = m_leftStick.getY();
      rightSpeed = m_rightStick.getY();
    }

    m_frontLeft.set(ControlMode.PercentOutput, leftSpeed);
    m_backLeft.set(ControlMode.PercentOutput, leftSpeed);
    m_frontRight.set(ControlMode.PercentOutput, rightSpeed);
    m_backRight.set(ControlMode.PercentOutput, rightSpeed);

    SmartDashboard.putNumber("AHRS Angle", ahrsAngle);

    }
    public double remap_range(double val, double old_min, double old_max, double new_min, double new_max){ // Basically just math to convert a value from an old range to 
      // new range (slope, line formula)
    return (new_min + (val - old_min)*((new_max - new_min)/(old_max - old_min)));
    }
    
}

