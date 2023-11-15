// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private WheelDrive backRight = new WheelDrive (8, 7, 4, 4);
  private WheelDrive backLeft = new WheelDrive (2, 1, 1, 3);
  private WheelDrive frontRight = new WheelDrive (6, 5, 3, 1);
  private WheelDrive frontLeft = new WheelDrive (4, 3, 2, 2);

  private SwerveDrive swerveDrive = new SwerveDrive (backRight, backLeft, frontRight, frontLeft);

  private Joystick leftJoy = new Joystick(0);
  private Joystick rightJoy = new Joystick(1);
  private XboxController xbox = new XboxController(2);

  private double leftJoyY;
  private double leftJoyX;
  private double rightJoyX;

  private PIDController zeroPID = new PIDController(0.01, 0, 0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    // m_chooser.addOption("My Auto", kCustomAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    leftJoyY = applyDeadband(leftJoy.getY(), 0.2);
    leftJoyX = applyDeadband(leftJoy.getX(), 0.2);
    rightJoyX = applyDeadband(rightJoy.getX(), 0.2);
    swerveDrive.drive(leftJoyX, leftJoyY, rightJoyX);
    SmartDashboard.putNumber("leftJoy X", leftJoyX);
    SmartDashboard.putNumber("leftJoy Y", leftJoyY);
    SmartDashboard.putNumber("rightJoy X", rightJoyX);

    if (xbox.getXButton()){
      swerveDrive.init();
    }
    if (xbox.getBButton()){
      swerveDrive.drive(0.1,0,0);
    }
    
    

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public static double applyDeadband(double value, double band){
    if(Math.abs(value) > Math.abs(band)){
      return value;
    }
    return 0;
    
  }
}
