// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private final VictorSPX m_frontLeft = new VictorSPX(1);
  private final VictorSPX m_backLeft = new VictorSPX(2);
  private final VictorSPX m_frontRight = new VictorSPX(3);
  private final VictorSPX m_backRight= new VictorSPX(4);

  private double leftSpeed = 0;
  private double rightSpeed = 0;

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRight.setInverted(true);
    m_backRight.setInverted(true);
    
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    leftSpeed = m_leftStick.getY();
    rightSpeed = m_rightStick.getY();
    m_frontLeft.set(ControlMode.PercentOutput, leftSpeed);
    m_backLeft.set(ControlMode.PercentOutput, leftSpeed);
    m_frontRight.set(ControlMode.PercentOutput, rightSpeed);
    m_backLeft.set(ControlMode.PercentOutput, rightSpeed);
    }
}
