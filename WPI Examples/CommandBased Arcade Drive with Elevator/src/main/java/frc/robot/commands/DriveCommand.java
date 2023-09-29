// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {  

  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> speedFunction;
  private final Supplier<Double> turnFunction;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speed, Supplier<Double> turn) {
    this.speedFunction = speed;
    this.turnFunction = turn;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = speedFunction.get() + turnFunction.get();
    double right = speedFunction.get() - turnFunction.get();
    driveSubsystem.setSpeeds(left, right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // driveSubsystem.setSpeeds(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
