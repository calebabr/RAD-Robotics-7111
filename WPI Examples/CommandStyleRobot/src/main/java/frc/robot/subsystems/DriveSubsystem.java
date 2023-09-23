// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final VictorSPX motorFrontLeft  = new VictorSPX(0);
  private final VictorSPX motorBackLeft   = new VictorSPX(1);
  private final VictorSPX motorFrontRight = new VictorSPX(2);
  private final VictorSPX motorBackRight  = new VictorSPX(3);
  public DriveSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
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
  }
    // This method will be called once per scheduler run
    public void setMotors(double speed, double rotation) {
      motorFrontLeft.set(VictorSPXControlMode.PercentOutput, speed + rotation / 2);
      motorBackLeft.set(VictorSPXControlMode.PercentOutput, speed + rotation / 2);
      motorFrontRight.set(VictorSPXControlMode.PercentOutput, speed - rotation / 2);
      motorBackRight.set(VictorSPXControlMode.PercentOutput, speed - rotation / 2);
    }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
