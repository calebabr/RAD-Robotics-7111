// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import javax.swing.plaf.RootPaneUI;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final PIDController gyroscope_pid = new PIDController(0.1, 0, 0);
  private Joystick leftStick;
  private Joystick rightStick;
  private XboxController m_xbox = new XboxController(2);
  private GenericEntry limelight_kP;
  private GenericEntry limelight_kI;
  private GenericEntry limelight_kD;
  private GenericEntry GyroMinMax;
  private float gyro_dead_range = 1;

  // Motors controlling driving
  private final CANSparkMax motorfrontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorbackLeft = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorfrontRight = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorbackRight= new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  // Cone/Cube Grabber parts
  public static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public static final Solenoid sol1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  public static final Solenoid sol2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  // limelight
  private final PhotonCamera limelight = new PhotonCamera("clockcam");

  private final PIDController limelight_pid = new PIDController(0.005, 0.00005, 0);

  // Joysticks
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private double turnSpeed = 0;

  // Gyroscope
  private double gyro_set_drive_speed = 0;
  
  private AHRS ahrs_gyroscope;

  private float ahrsAngle;
  @Override
  public void teleopInit() {
    // Setting up variables on the smart dashboard so we can change them while running the code.
    limelight_kP = Shuffleboard.getTab("SmartDashboard").add("kP", 0.01).withWidget("Text View").getEntry();
    limelight_kI = Shuffleboard.getTab("SmartDashboard").add("kI", 0.00005).withWidget("Text View").getEntry();
    limelight_kD = Shuffleboard.getTab("SmartDashboard").add("kD", 0).withWidget("Text View").getEntry();
    GyroMinMax = Shuffleboard.getTab("SmartDashboard").add("Gyroscope_deadzone", 1).withWidget("Text View").getEntry();
  }
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    ahrs_gyroscope = new AHRS(Port.kMXP);
    ahrs_gyroscope.reset();

    limelight_pid.setSetpoint(0);
    limelight_pid.setTolerance(10);

    motorfrontRight.setInverted(true);
    motorbackRight.setInverted(true);
    
    rightStick = new Joystick(0);
    leftStick = new Joystick(1);
  }

  @Override
  public void teleopPeriodic() {
    // Setting up gyroscope angles
    ahrsAngle = ahrs_gyroscope.getPitch();
    SmartDashboard.putNumber("AHRS Angle (Gyroscope)", ahrsAngle);

    // Start of code for the limelight
    double rotationSpeed;
    // Code to set variables for the limelight_pid, so we can tinker with them while it is running.
    limelight_pid.setP(limelight_kP.getDouble(0.005));
    limelight_pid.setI(limelight_kI.getDouble(0.0005));
    limelight_pid.setD(limelight_kD.getDouble(0));
    limelight_pid.setD(GyroMinMax.getDouble(1));
    if (m_xbox.getAButton()){
      var result = limelight.getLatestResult();
      if (result.hasTargets()) {
        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = -limelight_pid.calculate(result.getBestTarget().getYaw(), 0);
        leftSpeed = -rotationSpeed;
        rightSpeed = rotationSpeed;
        
      } else {
        // If we have no targets, stay still.
        rotationSpeed = 0;
    }
    // end of code for the limelight
      // Start code for gyroscope
    if (m_xbox.getBButton()){
      // if our gyroscope is in a certain range, we don't move.
      if (ahrsAngle < gyro_dead_range && ahrsAngle > -gyro_dead_range){
        gyro_set_drive_speed = 0;
      }
        // otherwise, we calculate where we need to move and set our left and right speed. 
      else
      {
        gyro_set_drive_speed = gyroscope_pid.calculate(ahrsAngle, 0); // this is bad, needs to use an encoder !!!
        leftSpeed = gyro_set_drive_speed;
        rightSpeed = gyro_set_drive_speed;
      }
    }
    leftSpeed = gyro_set_drive_speed;
    rightSpeed = gyro_set_drive_speed;
    // end gyroscope code

    // If we are not using gyroscope or limelight, then get joystick input for speed.
  } else {
      leftSpeed = leftStick.getY();
      rightSpeed = rightStick.getY();
      turnSpeed = leftStick.getX();
    }
    // Start Solenoid code, for grabber.
    if (m_xbox.getXButton()) {
      // if you press x, you are in cube grab mode, press again or cone grab mode to end.
      // if you press when a solenoid is already active, it resets it.
      if (sol1.get() == true || sol2.get() == true) {
        sol1.set(false);
        sol2.set(false);
      }
      else {
        sol1.set(true);
      }
    }
    if (m_xbox.getYButton()) {
      // this is cone grab mode, press again or cube grab mode to end.
      // if you press when a solenoid is already active, it resets it.
      if (sol1.get() == true || sol2.get() == true) {
        sol1.set(false);
        sol2.set(false);
      }
      else {
        sol1.set(true);
        sol2.set(true);
      }
    }
    // end solenoid code.

    // Setting the desired speed to the motors.
    motorfrontLeft.set(Math.pow(leftSpeed, 3));
    motorbackLeft.set(Math.pow(leftSpeed, 3));
    motorfrontRight.set(Math.pow(rightSpeed, 3));
    motorbackRight.set(Math.pow(rightSpeed, 3));

    SmartDashboard.putNumber("AHRS Angle (Gyroscope)", ahrsAngle);

    }
    public double remap_range(double val, double old_min, double old_max, double new_min, double new_max){ // Basically just math to convert a value from an old range to 
      // new range (slope, line formula)
    return (new_min + (val - old_min)*((new_max - new_min)/(old_max - old_min)));
    }
    
}