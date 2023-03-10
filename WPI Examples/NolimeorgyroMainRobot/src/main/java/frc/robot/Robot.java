// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;        
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import javax.swing.plaf.RootPaneUI;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import org.photonvision.PhotonCamera;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.filter.SlewRateLimiter;


/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private XboxController m_xbox = new XboxController(2);
  private static final int kEncoderPortA = 0;
  private static final int kEncoderPortB = 1;
  private final VictorSPX rotateMotor = new VictorSPX(1);
  private final CANSparkMax extendMotor = new CANSparkMax(5, MotorType.kBrushless);
  private double extendSpeed;
  private double rotateSpeed;
  private double rotateMaxValue = 5.0; // tinker!
  private double rotateMinValue = 6.0; // tinker!
  
  private RelativeEncoder arm_encoder;

  // Motors controlling 
  private final CANSparkMax motorFrontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorBackLeft = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorFrontRight = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorBackRight= new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  // Motor Controller 
  private final MotorControllerGroup left = new MotorControllerGroup(motorBackLeft, motorFrontLeft);
  private final MotorControllerGroup right = new MotorControllerGroup(motorBackRight, motorFrontRight);

  // 
  private DifferentialDrive robotDrive;
  
  // Cone/Cube Grabber 
  public static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public static final Solenoid sol1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  public static final Solenoid sol2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);

  // 
  private double ySpeed = 0;
  private double rSpeed = 0;
  private Joystick leftStick;
  private Joystick rightStick;
  private SlewRateLimiter rightJLimiter; // tinker?
  private SlewRateLimiter leftJLimiter;
  private SlewRateLimiter armLimiter;
  double rotationSpeed;


  @Override
  public void teleopInit() {
    arm_encoder.setPosition(0);
  }
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    

    motorFrontRight.setInverted(true);
    motorBackRight.setInverted(true);
    
    rightStick = new Joystick(0);
    leftStick = new Joystick(1);
    leftJLimiter = new SlewRateLimiter(0.5); // needs to be tested, tinker
    rightJLimiter = new SlewRateLimiter(0.5);
    armLimiter = new SlewRateLimiter(0.5);
    robotDrive = new DifferentialDrive(left, right);
  }

  @Override
  public void teleopPeriodic() {
    ySpeed = leftJLimiter.calculate(leftStick.getY());
    rSpeed = rightJLimiter.calculate(rightStick.getX());
    // Start Solenoid code, for grabber.
    if (m_xbox.getYButton()) {
      // this is cone grab mode, press again to end.
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

    // Start arm code
    rotateSpeed = armLimiter.calculate(m_xbox.getLeftY());
    extendSpeed = armLimiter.calculate(m_xbox.getRightY());
    // if (arm_encoder.getPosition() <= rotateMaxValue && arm_encoder.getPosition() >= rotateMinValue) // tinker with this encoder!
    // { 
      rotateMotor.set(VictorSPXControlMode.PercentOutput, rotateSpeed);
    // }
    // commented for now because we do not have encoders on our victor spx motors
    
    extendMotor.set(extendSpeed);

    // End arm code
    
  
    // Setting the desired speed to the motors.

    robotDrive.arcadeDrive(ySpeed, rSpeed);

    }
    public double remap_range(double val, double old_min, double old_max, double new_min, double new_max){ // Basically just math to convert a value from an old range to 
      // new range (slope, line formula)
    return (new_min + (val - old_min)*((new_max - new_min)/(old_max - old_min)));
    }
    
}