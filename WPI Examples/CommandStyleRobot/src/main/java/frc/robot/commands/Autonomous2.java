// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.DrivingConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;


import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class Autonomous2 extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem Drive_subsystem;
  private double speed;
  private double rotation;
  private double time;


  public Autonomous2(DriveSubsystem subsystem, double speed, double rotation, double time) {
    this.Drive_subsystem = subsystem;
    this.speed = speed;
    this.rotation = rotation;
    this.time = time;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Drive_subsystem.StartTimer();
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Drive_subsystem.GetTimer() <= time)
    {
      speed = 0.1;
      rotation = 0.2;
    }
    else
    {
      speed = 0;
      rotation = 0;
    }
    Drive_subsystem.setMotors(speed, rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drive_subsystem.StopTimer();
    Drive_subsystem.ResetTimer();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
