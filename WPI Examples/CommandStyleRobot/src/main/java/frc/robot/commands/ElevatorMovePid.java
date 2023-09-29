// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants.ElevatorConstantsPid;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;

/** An example command that uses an example subsystem. */
public class ElevatorMovePid extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ElevatorSubsystem Elevator_subsystem;
  private final double setpoint;
  private double movement;
  private double measure;
  private final PIDController pid = new PIDController(ElevatorConstantsPid.kp, ElevatorConstantsPid.ki, ElevatorConstantsPid.kd);

  public ElevatorMovePid(ElevatorSubsystem subsystem, double setpoint) {
    this.Elevator_subsystem = subsystem;
    this.setpoint = setpoint;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("ElevatorPID");
    measure = Elevator_subsystem.Encoder();
    movement = pid.calculate(measure, setpoint);
    Elevator_subsystem.MoveElevator(movement);
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
