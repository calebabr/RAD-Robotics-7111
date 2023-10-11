// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ElevatorSubsystem extends SubsystemBase {
  // private final TalonFX elevMotor = new TalonFX(11);
  // private final Encoder elevEncoder = new Encoder(0, 1);

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // public void setSpeeds(double speeds){
  //   elevMotor.set(TalonFXControlMode.PercentOutput,speeds);
  // }

  // public double getEncoderMeters() {
  //   return elevMotor.getSelectedSensorPosition() * 4; // constant that needs to be calculated to get tick to meters
  // }
}
