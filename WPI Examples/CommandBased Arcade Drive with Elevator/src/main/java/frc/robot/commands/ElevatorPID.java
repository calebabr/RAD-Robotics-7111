// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.math.controller.PIDController;

public class ElevatorPID extends CommandBase {

  private final ElevatorSubsystem elevatorSubsystem;
  private final PIDController elevPID;

  /** Creates a new ElevatorPID. */
  public ElevatorPID(ElevatorSubsystem elevSubsystem, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevSubsystem;
    this.elevPID = new PIDController(0, 0, 0);
    elevPID.setSetpoint(setPoint);
    addRequirements(elevSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setSpeeds(elevPID.calculate(setPoint));
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
