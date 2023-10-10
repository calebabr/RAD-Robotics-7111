// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX motor = new TalonFX(11);
  private double measure;
  
  /** Creates a new ExampleSubsystem. */

  
  public ElevatorSubsystem() {
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void MoveElevator(double speed) {
    motor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public double Encoder() {
    motor.setNeutralMode(NeutralMode.Brake);
    measure = motor.getSelectedSensorPosition();
    return measure;
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
