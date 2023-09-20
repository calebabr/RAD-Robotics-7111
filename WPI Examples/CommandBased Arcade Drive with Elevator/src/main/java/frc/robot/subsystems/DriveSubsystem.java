// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax backLeft = new CANSparkMax(0, MotorType.kBrushless);
  private final CANSparkMax backRight = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax frontRight = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax frontLeft = new CANSparkMax(3, MotorType.kBrushless);

  private final MotorControllerGroup left = new MotorControllerGroup(backLeft, frontLeft);
  private final MotorControllerGroup right = new MotorControllerGroup(backRight, frontRight);
  private final DifferentialDrive roboDrive = new DifferentialDrive(left, right);

  public DriveSubsystem() {
    backLeft.setInverted(true);
    frontLeft.setInverted(true);
  }

  public void setSpeeds(double speed, double rot){
   roboDrive.arcadeDrive(speed, rot);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
