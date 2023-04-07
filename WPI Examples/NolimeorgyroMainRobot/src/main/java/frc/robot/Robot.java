// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

// link to use: https://dev.studica.com/releases/2023/NavX.json
// import com.kauailabs.navx.frc.AHRS;
// import com.kauailabs.navx.frc.AHRS.BoardAxis;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;        
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

import edu.wpi.first.wpilibj.Timer;

import javax.lang.model.util.SimpleTypeVisitor14;
import javax.swing.plaf.RootPaneUI;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

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
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.revrobotics.CANSparkMax.IdleMode;
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

  private Timer autoTime;
  private int AutoState;
  private double autospeed;
  private double autoPast;

  private GenericEntry rotateArm;
  // private GenericEntry balRange;
  private GenericEntry armP;
  private GenericEntry armI;
  private GenericEntry armD;
  // private GenericEntry balP;
  // private GenericEntry balI;
  // private GenericEntry balD;

  private XboxController m_xbox = new XboxController(2);
  
  Timer clocka = new Timer();

  private final PIDController m_rightAutoPID = new PIDController(0.005, 0.00005, 0);
  private final PIDController m_leftAutoPID = new PIDController(0.005, 0.00005, 0);
  private final PIDController ArmAutoPID = new PIDController(0.00008, 0.0001, 0.00000001);
  private final PIDController ArmTelePID = new PIDController(0.00008, 0.0001, 0.00000001);
  private final PIDController balancePID = new PIDController(0.00019,  0.000002, 0.00001);

  // Motors controlling 
  private final CANSparkMax motorFrontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorBackLeft = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorFrontRight = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorBackRight= new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

  private final TalonFX rotateMotor = new TalonFX(10);
  private final CANSparkMax extendMotor = new CANSparkMax(5, MotorType.kBrushless);
  
  private final RelativeEncoder backLeftEncoder = motorBackLeft.getEncoder();
  private final RelativeEncoder frontLeftEncoder = motorFrontLeft.getEncoder();
  private final RelativeEncoder backRightEncoder = motorBackRight.getEncoder();
  private final RelativeEncoder frontRightEncoder = motorFrontRight.getEncoder();

  private final RelativeEncoder extendEncoder = extendMotor.getEncoder();


  private final VictorSPX clawRight = new VictorSPX(8);
  private final VictorSPX clawLeft = new VictorSPX(7);
  
  // Motor Controller 
  private final MotorControllerGroup left = new MotorControllerGroup(motorBackLeft, motorFrontLeft);
  private final MotorControllerGroup right = new MotorControllerGroup(motorBackRight, motorFrontRight);

  // 
  public DifferentialDrive robotDrive;
  
  // Cone/Cube Grabber 
  public static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public static final DoubleSolenoid sol1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
  public static final DoubleSolenoid sol2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

  // private AHRS ahrs;

  // 
  private double ySpeed = 0;
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private double rSpeed = 0;
  private double ahrsPitch;
  private double extendSpeed;
  private double rotateSpeed;
  private Joystick leftStick;
  private Joystick rightStick;
  private SlewRateLimiter rightJLimiter; // tinker?
  private SlewRateLimiter leftJLimiter;
  private SlewRateLimiter rotateLimiter;
  private SlewRateLimiter extendLimiter;

  double rotationSpeed;
  int currAngle;
  double range = 4;

  double currRotatePos = 0;
  double startRotatePos = 0;

  double currExtendPos = 0;
  double startExtendPos = 0;

  double currBRPos = 0;
  double currFRPos = 0;
  double currBLPos = 0;
  double currFLPos = 0;

  double startBRPos = 0;
  double startFRPos = 0;
  double startBLPos = 0;
  double startFLPos = 0;

  @Override
  public void robotInit() {
    motorFrontRight.setInverted(true);
    motorBackRight.setInverted(true);
    motorFrontLeft.setInverted(false);
    motorBackLeft.setInverted(false);
    
    clawLeft.setInverted(true);
    
    rightStick = new Joystick(1);
    leftStick = new Joystick(0);
    leftJLimiter = new SlewRateLimiter(1); 
    rightJLimiter = new SlewRateLimiter(0.95);
    rotateLimiter = new SlewRateLimiter(1.5);
    extendLimiter = new SlewRateLimiter(0.9);
    robotDrive = new DifferentialDrive(left, right);

    autoTime = new Timer();
    rotateMotor.setNeutralMode(NeutralMode.Brake);
    extendMotor.setIdleMode(IdleMode.kBrake);
    // ahrs = new AHRS(Port.kMXP);
    // ahrs.reset();
  }

  @Override
  public void robotPeriodic(){
    // ahrsPitch = ahrs.getPitch();

    // rotateArm = Shuffleboard.getTab("SmartDashboard").add("rotate", 2).withWidget("Text View").getEntry(); // tinker with this
    // balRange = Shuffleboard.getTab("SmartDashboard").add("balRange", 0).withWidget("Text View").getEntry(); // tinker with this

    //armP = Shuffleboard.getTab("SmartDashboard").add("arm P", 0.00008).withWidget("Text View").getEntry(); // tinker with this
    //armI = Shuffleboard.getTab("SmartDashboard").add("arm I", 0.0001).withWidget("Text View").getEntry(); // tinker with this
    //armD = Shuffleboard.getTab("SmartDashboard").add("arm D", 0.00000001).withWidget("Text View").getEntry(); // tinker with this
    
    // balP = Shuffleboard.getTab("SmartDashboard").add("bal P", 0.00019).withWidget("Text View").getEntry(); // tinker with this
    // balI = Shuffleboard.getTab("SmartDashboard").add("bal I", 0.000002).withWidget("Text View").getEntry(); // tinker with this
    // balD = Shuffleboard.getTab("SmartDashboard").add("bal D", 0.00001).withWidget("Text View").getEntry(); // tinker with this
    
    // range = balRange.getDouble(0);
  }

  @Override
  public void autonomousInit() {
    m_leftAutoPID.reset();
    m_rightAutoPID.reset();
    backLeftEncoder.setPosition(0);
    backRightEncoder.setPosition(0);
    frontLeftEncoder.setPosition(0);
    frontRightEncoder.setPosition(0);
    m_leftAutoPID.setSetpoint(4.25);
    m_rightAutoPID.setSetpoint(-4.25);
    autoPast = 0;  
    AutoState = 0;

    currRotatePos = rotateMotor.getSelectedSensorPosition();

    currExtendPos = extendEncoder.getPosition();

    currBRPos = backRightEncoder.getPosition();
    currFRPos = frontRightEncoder.getPosition();
    currBLPos = backLeftEncoder.getPosition();
    currFLPos = frontLeftEncoder.getPosition();
    
    startBRPos = backRightEncoder.getPosition();
    startFRPos = frontRightEncoder.getPosition();
    startBLPos = backLeftEncoder.getPosition();
    startFLPos = frontLeftEncoder.getPosition();

    startExtendPos = extendEncoder.getPosition();
    startRotatePos = rotateMotor.getSelectedSensorPosition();
    ySpeed = 0;
    rotateSpeed = 0;
    extendSpeed = 0;
    robotDrive.feed();
    ArmAutoPID.setPID(0.00008, 0.0001, 0.00000001);
  }

  /** This function is called periodically during autonomous. */


  @Override
  public void autonomousPeriodic() {
    currRotatePos = rotateMotor.getSelectedSensorPosition();
    currExtendPos = extendEncoder.getPosition();

    currBRPos = backRightEncoder.getPosition();
    currFRPos = frontRightEncoder.getPosition();
    currBLPos = backLeftEncoder.getPosition();
    currFLPos = frontLeftEncoder.getPosition();

    switch (AutoState) {

      // Rotate Arm Up
      case 0:
      /** 
      robotDrive.arcadeDrive(0, 0);
      if (currRotatePos < startRotatePos + 53371.9){ // rotate up to score mid
        rotateSpeed = 0.1 * ArmAutoPID.calculate(currRotatePos, startRotatePos + 53371.9);
        sol2.set(DoubleSolenoid.Value.kForward);
      }
      */
      if (currRotatePos < startRotatePos + 44000){ // rotate up to score mid
        rotateSpeed = 0.1 * ArmAutoPID.calculate(currRotatePos, startRotatePos + 44000);
        sol2.set(DoubleSolenoid.Value.kForward);
        extendMotor.set(0);
      }
      else{
        rotateSpeed = 0;
        sol2.set(DoubleSolenoid.Value.kReverse);
        autoTime.reset();
        autoTime.start();
        AutoState = 1;
      }
      rotateMotor.set(ControlMode.PercentOutput, rotateSpeed);
      SmartDashboard.putNumber("AutoCase", AutoState);

      break;
        
      // Spin Claw
      case 1:
      ySpeed = 0;   
      // HIGH SCORE: 
      /** 
      if (autoTime.hasElapsed(3.6)){
        clawRight.set(VictorSPXControlMode.PercentOutput,0); 
        clawLeft.set(VictorSPXControlMode.PercentOutput,0);
        extendMotor.set(0);
        ArmAutoPID.setPID(0.00008, 0.0001, 0.00000001);
        AutoState = 2;
      }
      else if (autoTime.hasElapsed(2.1)){ // retract for 1.5
        clawRight.set(VictorSPXControlMode.PercentOutput, 0); 
        clawLeft.set(VictorSPXControlMode.PercentOutput, 0);
        extendMotor.set(-0.5);
      }
      else if (autoTime.hasElapsed(1.5)){ // outtake for 0.6
        extendMotor.set(0);
        clawRight.set(VictorSPXControlMode.PercentOutput, 0.45); 
        clawLeft.set(VictorSPXControlMode.PercentOutput, 0.45);
      }
      else{ // extend OUT  for 1.5
        extendMotor.set(0.5);
      }
      */

      // MID SCORE:
      if (autoTime.hasElapsed(0.6)){
        clawRight.set(VictorSPXControlMode.PercentOutput,0); 
        clawLeft.set(VictorSPXControlMode.PercentOutput,0);
        ArmAutoPID.setPID(0.00008, 0.0001, 0.00000001);
        AutoState = 2;
      }
      else{ // outtake for 0.6
        clawRight.set(VictorSPXControlMode.PercentOutput, 0.45); 
        clawLeft.set(VictorSPXControlMode.PercentOutput, 0.45);
        extendMotor.set(0);
      }
      SmartDashboard.putNumber("AutoCase", AutoState);
      break;
      
      // Rotate arm down
      case 2:
      ySpeed = 0;
      if (currRotatePos > startRotatePos + 100){ // rotate down
        rotateSpeed = 0.1 * ArmAutoPID.calculate(currRotatePos, startRotatePos + 100);
        sol2.set(DoubleSolenoid.Value.kForward);
        extendMotor.set(0);

      }
      else{
        rotateSpeed = 0;
        sol2.set(DoubleSolenoid.Value.kReverse);
        autoTime.reset();
        autoTime.start();
        AutoState = 3;
      }
      rotateMotor.set(ControlMode.PercentOutput, rotateSpeed);
      break;
      
      // /** 
      // Back up out of community
      case 3:
      if (autoTime.hasElapsed(3.5)){
        ySpeed = 0;
      }
      else{
        ySpeed = 0.5;
        extendMotor.set(0);
      }      
      break;
      // */

      // CODE TO DRIVE UP AND BALANCE
      /** 
      case 3: 
      if (autoTime.hasElapsed(1.5)){
        ySpeed = 0;
        AutoState = 4;
      }
      else{
        ySpeed = 0.4;
      }
      break;
      */

      // BALANCE
      case 4: 
      if (Math.abs(ahrsPitch) < 2){
        ySpeed = 0;
        AutoState = 5;
      }
      else{
        ySpeed = balancePID.calculate(ahrsPitch, 0);
      }
  }

  SmartDashboard.putNumber("AutoCase", AutoState);
  SmartDashboard.putNumber("rotate curr pos", currRotatePos);
  SmartDashboard.putNumber("rotate start pos", startRotatePos);
  SmartDashboard.putNumber("rotate Speed", rotateSpeed);
  SmartDashboard.putNumber("y speed", ySpeed);

  robotDrive.arcadeDrive(ySpeed, 0);
}



  @Override
  public void teleopInit() {
    

    // arm_encoder.setPosition(0);
    startExtendPos = extendEncoder.getPosition();
    startRotatePos = rotateMotor.getSelectedSensorPosition();
    
    ArmTelePID.setPID(0.00008, 0.0001, 0.00000001);
    // balancePID.setPID(balP.getDouble(0.00019), balI.getDouble(0.000002),balD.getDouble(0.00001));

    rotateSpeed = 0;
  }
 

  @Override
  public void teleopPeriodic() {
    currRotatePos = rotateMotor.getSelectedSensorPosition();
    currExtendPos = extendEncoder.getPosition();
    // balancePID.setPID(balP.getDouble(0.00019), balI.getDouble(0.000002),balD.getDouble(0.00001));
    ArmTelePID.setPID(0.00008, 0.0001, 0.00000001);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//  CLAW CODE
    // Claw Solenoids using X and Y to open and close
    if (m_xbox.getYButtonPressed()) { // grabber
      sol1.set(DoubleSolenoid.Value.kForward);
      // sol2.set(DoubleSolenoid.Value.kForward);
    }
    else if (m_xbox.getXButtonPressed()){
      sol1.set(DoubleSolenoid.Value.kReverse);
      // sol2.set(DoubleSolenoid.Value.kReverse);
    }

    // Claw Motors using A and B to intake and outtake
    if (m_xbox.getBButton()){ // outtake game piece
      clawRight.set(VictorSPXControlMode.PercentOutput, 0.5); 
      clawLeft.set(VictorSPXControlMode.PercentOutput, 0.5);
    }
    else if (m_xbox.getAButton()){ // intake game piece
      clawRight.set(VictorSPXControlMode.PercentOutput, -0.5); 
      clawLeft.set(VictorSPXControlMode.PercentOutput, -0.5);
    }
    else{
      clawRight.set(VictorSPXControlMode.PercentOutput, 0);
      clawLeft.set(VictorSPXControlMode.PercentOutput, 0);
    }
    
    // end solenoid code.
    
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    ROTATION CODE
      if (m_xbox.getRightTriggerAxis() > 0.07 && m_xbox.getLeftTriggerAxis() < 0.07){ // rotate up
        sol2.set(DoubleSolenoid.Value.kForward);
        rotateSpeed = 0.1 * 2; 
      }
      else if (m_xbox.getRightTriggerAxis() < 0.07 && m_xbox.getLeftTriggerAxis() > 0.07){ // rotate down
        sol2.set(DoubleSolenoid.Value.kForward);
        rotateSpeed = -0.07 * 2; 
      }
      else if (m_xbox.getPOV(0) == 0){ // high cube and mid cone preset using up on the d-pad
        if (currRotatePos < startRotatePos + 53371.9){
          rotateSpeed = 0.1* ArmTelePID.calculate(currRotatePos, startRotatePos + 53371.9);
          sol2.set(DoubleSolenoid.Value.kForward);
        }
        else{
          rotateSpeed = 0;
          sol2.set(DoubleSolenoid.Value.kReverse);
        }
      }
      else if (m_xbox.getPOV(0) == 90){ // mid cube preset using right on the d-pad
        if (currRotatePos < startRotatePos + 44000){
          rotateSpeed = 0.1 * ArmTelePID.calculate(currRotatePos, startRotatePos + 44000);
          sol2.set(DoubleSolenoid.Value.kForward);
        }
        else{
          rotateSpeed = 0;
          sol2.set(DoubleSolenoid.Value.kReverse);
        }
      }
      else if (m_xbox.getPOV(0) == 180){ // low or resting position using down on the d-pad
        if (currRotatePos > startRotatePos + 100){
          rotateSpeed = 0.1 * ArmTelePID.calculate(currRotatePos, startRotatePos + 100);
          sol2.set(DoubleSolenoid.Value.kForward);
        }
        else{
          rotateSpeed = 0;
          sol2.set(DoubleSolenoid.Value.kReverse);  
        }
      }
      else if (m_xbox.getPOV(0) == 270){ // high cone using left on the d-pad
        if (currRotatePos < startRotatePos + 60000){
          rotateSpeed = 0.1 * ArmTelePID.calculate(currRotatePos, startRotatePos + 60000);
          sol2.set(DoubleSolenoid.Value.kForward);
        }
        else{
          rotateSpeed = 0;
          sol2.set(DoubleSolenoid.Value.kReverse);
        }
      }
      else{
        sol2.set(DoubleSolenoid.Value.kReverse);
        rotateSpeed = 0;
      }
      rotateMotor.set(ControlMode.PercentOutput, rotateSpeed);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//    EXTENSION CODE
      if (m_xbox.getRightBumper()){ // extend using right bumper
        extendSpeed = 0.7;
        
      }
      else if (m_xbox.getLeftBumper()){ // retract using left bumper
        extendSpeed = -0.7;
      }
      else{
        extendSpeed = 0;
      }
      extendMotor.set(extendSpeed); 
  
/////////////////////////////////////////////////////////////////////////////////////////////////

  // DRIVE CODE
  // if (rightStick.getTrigger()){ // if holding right stick trigger, balance in teleop
    // ySpeed = balancePID.calculate(ahrsPitch, range); // PID controller, ahrsPitch is current, range is target
  // }
  // else{
    ySpeed = leftJLimiter.calculate(rightStick.getY()) * 0.85;
  // }
  rSpeed = rightJLimiter.calculate(leftStick.getX()) * 0.75;
  robotDrive.arcadeDrive(ySpeed, rSpeed);


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Dashboard Values
  SmartDashboard.putNumber("Left J", ySpeed);
  SmartDashboard.putNumber("Right J", rSpeed);
  SmartDashboard.putNumber("CurrPitch", currAngle);
  SmartDashboard.putNumber("rotate curr pos", currRotatePos);
  SmartDashboard.putNumber("rotate start pos", startRotatePos);
  SmartDashboard.putNumber("xbox dpad", m_xbox.getPOV(0));
  // SmartDashboard.putNumber("AHRS Pitch", ahrsPitch);
}

    // END ROBOT OPERATION CODE
    //////////////////////////////////////////////////////////////////////////////
    public double remap_range(double val, double old_min, double old_max, double new_min, double new_max){ // Basically just math to convert a value from an old range to 
      // new range (slope, line formula)
    return (new_min + (val - old_min)*((new_max - new_min)/(old_max - old_min)));
    }
  }
