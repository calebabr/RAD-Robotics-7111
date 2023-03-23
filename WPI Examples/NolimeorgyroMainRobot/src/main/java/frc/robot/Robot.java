// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

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

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.filter.SlewRateLimiter;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Point;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {

  private Timer autoTime;

  private GenericEntry gyro_kP;
  private GenericEntry gyro_kI;
  private GenericEntry gyro_kD;
  Thread m_visionThread;
  private XboxController m_xbox = new XboxController(2);
  // private static final int kEncoderPortA = 0;
  // private static final int kEncoderPortB = 1;
  private final CANSparkMax rotateMotor = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax extendMotor = new CANSparkMax(5, MotorType.kBrushless);
  private double extendSpeed;
  private double rotateSpeed;
  private double rotateMaxValue = 5.0; // tinker!
  private double rotateMinValue = 6.0; // tinker!
  Timer clocka = new Timer();
  private final PIDController m_rightAutoPID = new PIDController(0.005, 0.00005, 0);
  private final PIDController m_leftAutoPID = new PIDController(0.005, 0.00005, 0);


  // private AHRS gyro;
  private PIDController gyroPID;


  // private RelativeEncoder arm_encoder;

  // Motors controlling 
  private final CANSparkMax motorFrontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorBackLeft = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorFrontRight = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorBackRight= new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  private final RelativeEncoder backLeftEncoder = motorBackLeft.getEncoder();
  private final RelativeEncoder frontLeftEncoder = motorFrontLeft.getEncoder();
  private final RelativeEncoder backRightEncoder = motorBackRight.getEncoder();
  private final RelativeEncoder frontRightEncoder = motorFrontRight.getEncoder();




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
  // public static final DoubleSolenoid sol2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);


  // 
  private double ySpeed = 0;
  private double leftSpeed = 0;
  private double rightSpeed = 0;
  private double rSpeed = 0;
  private Joystick leftStick;
  private Joystick rightStick;
  private SlewRateLimiter rightJLimiter; // tinker?
  private SlewRateLimiter leftJLimiter;
  private SlewRateLimiter rotateLimiter;
  private SlewRateLimiter extendLimiter;
  double rotationSpeed;
  int currAngle;


  @Override
  public void robotInit() {
    // gyroPID = new PIDController(0.5, 0, 0);
        // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_visionThread =
        new Thread(
            () -> {
              // Get the UsbCamera from CameraServer
              UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
              camera.setResolution(640, 480);

              // Get a CvSink. This will capture Mats from the camera
              CvSink cvSink = CameraServer.getVideo();
              // Setup a CvSource. This will send images back to the Dashboard
              CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

              // Mats are very memory expensive. Lets reuse this Mat.
              Mat mat = new Mat();

              // This cannot be 'true'. The program will never exit if it is. This
              // lets the robot stop this thread when restarting robot code or
              // deploying.
              while (!Thread.interrupted()) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (cvSink.grabFrame(mat) == 0) {
                  // Send the output the error.
                  outputStream.notifyError(cvSink.getError());
                  // skip the rest of the current iteration
                  continue;
                }
                // Put a rectangle on the image
                Imgproc.rectangle(
                    mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);
                // Give the output stream a new image to display
                outputStream.putFrame(mat);
              }
            });
    m_visionThread.setDaemon(true);
    m_visionThread.start();

    motorFrontRight.setInverted(true);
    motorBackRight.setInverted(true);
    motorFrontLeft.setInverted(false);
    motorBackLeft.setInverted(false);
    
    clawLeft.setInverted(true);
    
    rightStick = new Joystick(1);
    leftStick = new Joystick(0);
    leftJLimiter = new SlewRateLimiter(1); // needs to be tested, tinker
    rightJLimiter = new SlewRateLimiter(0.95);
    rotateLimiter = new SlewRateLimiter(1.5);
    extendLimiter = new SlewRateLimiter(0.9);
    robotDrive = new DifferentialDrive(left, right);

    //gyro = new AHRS(SPI.Port.kMXP);
    // gyro.reset();

    autoTime = new Timer();

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

    
  }

  /** This function is called periodically during autonomous. */


  @Override
  public void autonomousPeriodic() {
    autoTime.start();
    if (autoTime.hasElapsed(3.5)){
      rightSpeed = 0;
      leftSpeed = 0;
    }
    else{
      rightSpeed = 0.45;
      leftSpeed = 0.45;
    }
    robotDrive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void teleopInit() {
    // autoTime.stop();
    gyro_kP = Shuffleboard.getTab("SmartDashboard").add("kP", 0).withWidget("Text View").getEntry(); // tinker with this
    gyro_kI = Shuffleboard.getTab("SmartDashboard").add("kI", 0).withWidget("Text View").getEntry(); // tinker with this
    gyro_kD = Shuffleboard.getTab("SmartDashboard").add("kD", 0).withWidget("Text View").getEntry(); // tinker with this
    // arm_encoder.setPosition(0);
  }
 

  @Override
  public void teleopPeriodic() {
    // gyroPID.setP(gyro_kP.getDouble(0));
    // gyroPID.setI(gyro_kI.getDouble(0));
    // gyroPID.setD(gyro_kD.getDouble(0));
    // currAngle = gyro.getBoardYawAxis().board_axis.getValue();


    
    // Start Solenoid code, for grabber.
    if (m_xbox.getYButtonPressed()) {
      sol1.set(DoubleSolenoid.Value.kForward);
      // sol2.set(DoubleSolenoid.Value.kForward);
    }
    else if (m_xbox.getXButtonPressed()){
      sol1.set(DoubleSolenoid.Value.kReverse);
      // sol2.set(DoubleSolenoid.Value.kReverse);
    }

    // Claw Motors
    if (m_xbox.getRightBumper()){ // suck in game piece
      clawRight.set(VictorSPXControlMode.PercentOutput, 0.5); 
      clawLeft.set(VictorSPXControlMode.PercentOutput, 0.5);
    }
    else if (m_xbox.getLeftBumper()){ // spit out game piece
      clawRight.set(VictorSPXControlMode.PercentOutput, -0.5); 
      clawLeft.set(VictorSPXControlMode.PercentOutput, -0.5);
    }
    else{
      clawRight.set(VictorSPXControlMode.PercentOutput, 0);
      clawLeft.set(VictorSPXControlMode.PercentOutput, 0);
    }

    // end solenoid code.

    // Start arm code
    // rotateSpeed = rotateLimiter.calculate(m_xbox.getLeftY());
    // if (arm_encoder.getPosition() <= rotateMaxValue && arm_encoder.getPosition() >= rotateMinValue) // tinker with this encoder!
    // { 
    // }
    // commented for now because we do not have encoders on our victor spx motors
    SmartDashboard.putNumber("Extend Motor", extendSpeed);
    // if (m_xbox.getAButton()){
      // extendMotor.set(0.5);
    // }
    // else if (m_xbox.getBButton()){
      // extendMotor.set(-0.5);
    // }
    // else{
      // extendMotor.set(0);
    // }
    
    if (m_xbox.getAButton()){
      if (m_xbox.getRightTriggerAxis() < 0.04 && m_xbox.getLeftTriggerAxis() < 0.04){ // deadzone 
        rotateSpeed = 0; 
      }
      else if (m_xbox.getRightTriggerAxis() > 0.04 && m_xbox.getLeftTriggerAxis() < 0.04){ // rotate up
        rotateSpeed = -rotateLimiter.calculate(m_xbox.getRightTriggerAxis()) * 0.2; // m_xbox.getRightTriggerAxis(), use right
      }
      else if (m_xbox.getRightTriggerAxis() < 0.04 && m_xbox.getLeftTriggerAxis() > 0.04){ // rotate down
        rotateSpeed = rotateLimiter.calculate(m_xbox.getLeftTriggerAxis()) * 0.4; // m_xbox.getLeftTriggerAxis(), use left
      }
      rotateMotor.set(rotateSpeed);
    }
    
    if (m_xbox.getBButton()){
      if (m_xbox.getRightTriggerAxis() < 0.04 && m_xbox.getLeftTriggerAxis() < 0.04){ // deadzone 
        extendSpeed = 0; 
      }
      else if (m_xbox.getRightTriggerAxis() > 0.04 && m_xbox.getLeftTriggerAxis() < 0.04){ // extend
        extendSpeed = extendLimiter.calculate(m_xbox.getRightTriggerAxis()); // use right
      }
      else if (m_xbox.getRightTriggerAxis() < 0.04 && m_xbox.getLeftTriggerAxis() > 0.04){ // retract
        extendSpeed = -extendLimiter.calculate(m_xbox.getLeftTriggerAxis()); // use left
      }
      extendMotor.set(extendSpeed);
    }

    // End arm code
    
  
    // Setting the desired speed to the motors.
    SmartDashboard.putNumber("Left J", ySpeed);
    SmartDashboard.putNumber("Right J", rSpeed);
    SmartDashboard.putNumber("CurrPitch", currAngle);
    
    //if(leftStick.getTrigger()){
      //ySpeed = gyroPID.calculate(currAngle, 0);
    //}
    //else{
      ySpeed = leftJLimiter.calculate(rightStick.getY()) * 0.85;
    // }
      rSpeed = rightJLimiter.calculate(leftStick.getX()) * 0.70;
      robotDrive.arcadeDrive(ySpeed, rSpeed);
  }////////////////////////////////////////////
    public double remap_range(double val, double old_min, double old_max, double new_min, double new_max){ // Basically just math to convert a value from an old range to 
      // new range (slope, line formula)
    return (new_min + (val - old_min)*((new_max - new_min)/(old_max - old_min)));
    }
    
}