// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final VictorSPX backLeft = new VictorSPX(1);
  private final VictorSPX backRight = new VictorSPX(2);
  private final VictorSPX frontRight = new VictorSPX(3);
  private final VictorSPX frontLeft = new VictorSPX(4);



  public DriveSubsystem() {
    backLeft.setInverted(true);
    frontLeft.setInverted(true);
  }

  public void setSpeeds(double left, double right){
    backLeft.set(VictorSPXControlMode.PercentOutput, left);
    frontLeft.set(VictorSPXControlMode.PercentOutput, left);
    backRight.set(VictorSPXControlMode.PercentOutput, right);
    frontRight.set(VictorSPXControlMode.PercentOutput, right);

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
