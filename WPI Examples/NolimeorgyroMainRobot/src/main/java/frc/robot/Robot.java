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
import edu.wpi.first.wpilibj.motorcontrol.Victor;

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
  
  Thread m_visionThread;
  private XboxController m_xbox = new XboxController(2);
  // private static final int kEncoderPortA = 0;
  // private static final int kEncoderPortB = 1;
  private final VictorSPX rotateMotor = new VictorSPX(9);
  private final CANSparkMax extendMotor = new CANSparkMax(5, MotorType.kBrushless);
  private double extendSpeed;
  private double rotateSpeed;
  private double rotateMaxValue = 5.0; // tinker!
  private double rotateMinValue = 6.0; // tinker!
  
  // private RelativeEncoder arm_encoder;

  // Motors controlling 
  private final CANSparkMax motorFrontLeft = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorBackLeft = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorFrontRight = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final CANSparkMax motorBackRight= new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  
  private final VictorSPX clawLeft = new VictorSPX(8);
  private final VictorSPX clawRight = new VictorSPX(7);
  
  // Motor Controller 
  private final MotorControllerGroup left = new MotorControllerGroup(motorBackLeft, motorFrontLeft);
  private final MotorControllerGroup right = new MotorControllerGroup(motorBackRight, motorFrontRight);

  // 
  private DifferentialDrive robotDrive;
  
  // Cone/Cube Grabber 
  public static final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public static final Solenoid sol1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
  public static final Solenoid sol2 = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  public static final Solenoid sol3 = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
  public static final Solenoid sol4 = new Solenoid(PneumaticsModuleType.CTREPCM, 3);

  // 
  private double ySpeed = 0;
  private double rSpeed = 0;
  private Joystick leftStick;
  private Joystick rightStick;
  private SlewRateLimiter rightJLimiter; // tinker?
  private SlewRateLimiter leftJLimiter;
  private SlewRateLimiter rotateLimiter;
  private SlewRateLimiter extendLimiter;
  double rotationSpeed;


  @Override
  public void teleopInit() {
    // arm_encoder.setPosition(0);
  }
  @Override
  public void robotInit() {
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
    
    rightStick = new Joystick(0);
    leftStick = new Joystick(1);
    leftJLimiter = new SlewRateLimiter(0.5); // needs to be tested, tinker
    rightJLimiter = new SlewRateLimiter(0.5);
    rotateLimiter = new SlewRateLimiter(0.5);
    extendLimiter = new SlewRateLimiter(0.5);
    robotDrive = new DifferentialDrive(left, right);
  }

  @Override
  public void teleopPeriodic() {
    ySpeed = leftJLimiter.calculate(leftStick.getY());
    rSpeed = rightJLimiter.calculate(rightStick.getY());
    // Start Solenoid code, for grabber.
    if (m_xbox.getYButton()) {
      // this is cone grab mode, press again to end.
      // if you press when a solenoid is already active, it resets it.
      if (sol2.get() == true || sol3.get() == true) {
        sol1.set(true);
        sol2.set(false);
        sol3.set(false);
        sol4.set(true);
      }
      else {
        sol1.set(false);
        sol2.set(true);
        sol3.set(true);
        sol4.set(false);
      }
    }
    if (m_xbox.getRightBumper()){
      clawLeft.set(VictorSPXControlMode.PercentOutput, 0.5);
      clawRight.set(VictorSPXControlMode.PercentOutput, 0.5);
    }
    if (m_xbox.getLeftBumper()){
      clawLeft.set(VictorSPXControlMode.PercentOutput, -0.5);
      clawRight.set(VictorSPXControlMode.PercentOutput, -0.5);
    }
    else{
      clawLeft.set(VictorSPXControlMode.PercentOutput, 0);
      clawRight.set(VictorSPXControlMode.PercentOutput, 0);
    }

    // end solenoid code.

    // Start arm code
    rotateSpeed = rotateLimiter.calculate(m_xbox.getLeftY());
    extendSpeed = extendLimiter.calculate(m_xbox.getRightY());
    // if (arm_encoder.getPosition() <= rotateMaxValue && arm_encoder.getPosition() >= rotateMinValue) // tinker with this encoder!
    // { 
      rotateMotor.set(VictorSPXControlMode.PercentOutput, rotateSpeed);
    // }
    // commented for now because we do not have encoders on our victor spx motors
    
    extendMotor.set(Math.pow(extendSpeed, 3));

    // End arm code
    
  
    // Setting the desired speed to the motors.
    SmartDashboard.putNumber("Left J", ySpeed);
    SmartDashboard.putNumber("Right J", rSpeed);

    robotDrive.tankDrive(ySpeed, rSpeed);

    }
    public double remap_range(double val, double old_min, double old_max, double new_min, double new_max){ // Basically just math to convert a value from an old range to 
      // new range (slope, line formula)
    return (new_min + (val - old_min)*((new_max - new_min)/(old_max - old_min)));
    }
    
}