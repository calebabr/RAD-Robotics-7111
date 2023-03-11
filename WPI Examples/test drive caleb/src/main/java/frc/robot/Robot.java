// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.filter.SlewRateLimiter;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;

  private double xSpeed;
  private double rotateSpeed;
  private double leftSpeed;

  private final MotorController m_frontLeftMotor = new PWMSparkMax(0);
  private final MotorController m_frontRightMotor = new PWMSparkMax(1);
  private final MotorController m_backLeftMotor = new PWMSparkMax(2);
  private final MotorController m_backRightMotor = new PWMSparkMax(3);

  private final MotorControllerGroup left = new MotorControllerGroup(m_backLeftMotor, m_frontLeftMotor);
  private final MotorControllerGroup right = new MotorControllerGroup(m_backRightMotor, m_frontRightMotor);

 private final SlewRateLimiter leftFilter = new SlewRateLimiter(0.5);
 private final SlewRateLimiter rightFilter = new SlewRateLimiter(0.5);

  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRightMotor.setInverted(true);

    m_myRobot = new DifferentialDrive(left, right);
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);

    xSpeed = 0;
    rotateSpeed = 0;
    leftSpeed = 0;
  }

  @Override
  public void teleopPeriodic() {
    xSpeed = leftFilter.calculate(m_leftStick.getY());
    rotateSpeed = rightFilter.calculate(m_rightStick.getX());
    m_myRobot.arcadeDrive(xSpeed, rotateSpeed);

  }
}
