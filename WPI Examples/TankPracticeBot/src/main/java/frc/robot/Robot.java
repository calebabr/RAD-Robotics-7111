// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import javax.swing.plaf.RootPaneUI;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.math.controller.PIDController;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick m_leftStick;
  private Joystick m_rightStick;
  private XboxController m_xbox = new XboxController(2);
  private GenericEntry kP;
  private GenericEntry kI;
  private GenericEntry kD;
  private GenericEntry MinMax;
  private float range = 20;
  private final VictorSPX m_frontLeft = new VictorSPX(1);
  private final VictorSPX m_backLeft = new VictorSPX(2);
  private final VictorSPX m_frontRight = new VictorSPX(3);
  private final VictorSPX m_backRight= new VictorSPX(4);
  //public static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  //public static final Solenoid sol1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  //public static final Solenoid sol2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  //private final PhotonCamera m_camera = new PhotonCamera("clockcam");

  private final PIDController m_pid = new PIDController(0.01, 0.00005, 0.001);

  private double leftSpeed = 0;
  private double rightSpeed = 0;

  private double speed = 0;
  
  private AHRS ahrs;

  private float ahrsAngle;
  @Override
  public void teleopInit() {
    kP = Shuffleboard.getTab("SmartDashboard").add("kP Gyro", 0.01).withWidget("Text View").getEntry();
    kI = Shuffleboard.getTab("SmartDashboard").add("kI Gyro", 0.00005).withWidget("Text View").getEntry();
    kD = Shuffleboard.getTab("SmartDashboard").add("kD Gyro", 0.001).withWidget("Text View").getEntry();
    MinMax = Shuffleboard.getTab("SmartDashboard").add("Min to Max deadzone", 1).withWidget("Text View").getEntry();
  }
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
    SmartDashboard.putNumber("AHRS Angle", ahrsAngle);
    m_pid.setP(kP.getDouble(0.005));
    m_pid.setI(kI.getDouble(0.0005));
    m_pid.setD(kD.getDouble(0));
    m_pid.setD(MinMax.getDouble(5));
    /*elif (m_xbox.getAButton()){
      var result = m_camera.getLatestResult();
      if (result.hasTargets()) {
        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -m_pid.calculate(result.getBestTarget().getYaw(), 0);
        leftSpeed = m_leftStick.getY();
        rightSpeed = m_rightStick.getY();
        m_myRobot.tankDrive(leftSpeed + (rotationSpeed / 2),rightSpeed - (rotationSpeed / 2));
      } else {
        // If we have no targets, stay still.
        rotationSpeed = 0;
      }
      leftSpeed = speed;
      rightSpeed = speed;
    }
    else{*/

     /*}
      
    */
    leftSpeed = m_leftStick.getY();
    rightSpeed = m_rightStick.getY();
    if (m_xbox.getBButton()){
      if ( Math.abs(ahrsAngle) < range) {}
      else {
        speed = m_pid.calculate(ahrsAngle, 0);
        leftSpeed = speed;
        rightSpeed = speed;
      }
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