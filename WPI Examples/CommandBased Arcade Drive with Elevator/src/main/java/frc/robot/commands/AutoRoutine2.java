// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoRoutine2 extends CommandBase {
  private final DriveSubsystem driveSub;
  private final double timeRun;
  private final double leftSpeed;
  private final double rightSpeed;
  /** Creates a new AutoRoutine2. */
  public AutoRoutine2(DriveSubsystem driveSubsystem, double timer, double LSpeed, double RSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSub = driveSubsystem;
    this.timeRun = timer;
    this.leftSpeed = LSpeed;
    this.rightSpeed = RSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (driveSub.getAutoTime() < timeRun){
      driveSub.setSpeeds(leftSpeed, rightSpeed);
    }
    else{
      driveSub.setSpeeds(0, 0);
    }
    driveSub.addAutoTime();
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
