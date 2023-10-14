// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRoutine1 extends CommandBase {
  private final DriveSubsystem driveSub;
  private final double timeRun;
  private final double speed;

  /** Creates a new AutoRoutine1. */
  public AutoRoutine1(DriveSubsystem driveSubsystem, double timer, double speed) { 
    // timer is max time auto will be run for
    this.driveSub = driveSubsystem; 
    this.timeRun = timer;
    this.speed = speed;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSub.startTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveSub.getTime() < timeRun){
      driveSub.setSpeeds(speed, speed);
    }
    else{
      driveSub.setSpeeds(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSub.resetTime();
    driveSub.stopTime();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
