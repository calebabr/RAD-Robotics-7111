// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.DrivingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class DriveForward extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem Drive_subsystem;
  private Supplier<Double> speed;
  private Supplier<Double> rotation;
  private double new_speed;
  private double new_rotation;


  public DriveForward(DriveSubsystem subsystem, Supplier<Double> speed, Supplier<Double> rotation) {
    this.Drive_subsystem = subsystem;
    this.speed = speed;
    this.rotation = rotation;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Driving");
    if (Math.abs(rotation.get()) < DrivingConstants.DeadZone) {
        new_rotation = 0;
    }
    if (Math.abs(speed.get()) < DrivingConstants.DeadZone) {
      new_speed = 0;
    }
    Drive_subsystem.setMotors(new_speed, new_rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
