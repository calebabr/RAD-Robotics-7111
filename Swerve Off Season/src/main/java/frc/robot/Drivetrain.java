// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // Change for our robot
  private final Translation2d m_frontLeftLocation = new Translation2d(-0.288925, 0.288925); // change 11.375 inches to meters
  private final Translation2d m_frontRightLocation = new Translation2d(0.288925, 0.288925);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.288925, -0.288925);
  private final Translation2d m_backRightLocation = new Translation2d(0.288925, -0.288925);

  // Change ID Numbers of each Speed Controller and Encoder channels
  public final SwerveModule m_frontLeft = new SwerveModule(3, 4, 2);
  public final SwerveModule m_frontRight = new SwerveModule(5, 6, 3);
  public final SwerveModule m_backLeft = new SwerveModule(1, 2, 1);
  public final SwerveModule m_backRight = new SwerveModule(7, 8, 4);

  // Test if AHRS gyro can use generic FRC AnalogGyro class
  private final AnalogGyro m_gyro = new AnalogGyro(0); // arbitrary port, check on RoboRio
  public final AHRS ahrsGyro = new AHRS(Port.kMXP);
  
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(m_kinematics, ahrsGyro.getRotation2d(), new SwerveModulePosition[] { 
        m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()
      });
  
  public Drivetrain() {
    m_gyro.reset();
    // m_frontLeft.zeroWheel();
    // m_frontRight.zeroWheel();
    // m_backLeft.zeroWheel();
    // m_backRight.zeroWheel();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] { 
          m_frontLeft.getPosition(), m_frontRight.getPosition(), m_backLeft.getPosition(), m_backRight.getPosition()
        });
  }

  // public void zeroTrain(){
  //   m_frontLeft.zeroWheel();
  //   m_frontRight.zeroWheel();
  //   m_backLeft.zeroWheel();
  //   m_backRight.zeroWheel();
  // }
}