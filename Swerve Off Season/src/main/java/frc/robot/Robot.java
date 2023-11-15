// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);
  private final Joystick joyLeft = new Joystick(0);
  private final Joystick joyRight = new Joystick(1);
  private final Drivetrain m_swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit(){
    // m_swerve.zeroTrain();
  }

  @Override
  public void robotPeriodic(){
    // SmartDashboard.putNumber("FL Turn Pos", m_swerve.m_frontLeft.getTurn());
    SmartDashboard.putNumber("FL Abs Turn Pos", m_swerve.m_frontLeft.getAbsTurn());

    // SmartDashboard.putNumber("FR Turn Pos", m_swerve.m_frontRight.getTurn());
    SmartDashboard.putNumber("FR Abs Turn Pos", m_swerve.m_frontRight.getAbsTurn());

    // SmartDashboard.putNumber("BL Turn Pos", m_swerve.m_backLeft.getTurn());
    SmartDashboard.putNumber("BL Abs Turn Pos", m_swerve.m_backLeft.getAbsTurn());

    // SmartDashboard.putNumber("BR Turn Pos", m_swerve.m_backRight.getTurn());
    SmartDashboard.putNumber("BR Abs Turn Pos", m_swerve.m_backRight.getAbsTurn());

    SmartDashboard.putNumber("Gyro Degrees", m_swerve.ahrsGyro.getRotation2d().getDegrees());
    SmartDashboard.putNumber("Gyro Rotations", m_swerve.ahrsGyro.getRotation2d().getRotations());
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    m_swerve.updateOdometry();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);

    // if (m_controller.getAButtonPressed()){
      // m_swerve.zeroTrain();
    // }
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(joyLeft.getX(), 0.2))
            * Drivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(joyLeft.getY(), 0.2))
            * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(joyRight.getX(), 0.2))
            * Drivetrain.kMaxAngularSpeed;

    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}
