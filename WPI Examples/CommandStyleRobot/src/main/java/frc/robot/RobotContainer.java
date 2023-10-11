// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.ElevatorConstantsPid;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ElevatorMoveDown;
import frc.robot.commands.ElevatorMovePid;
import frc.robot.commands.ElevatorMoveUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(2);
  private final Joystick joystick1 = new Joystick(0);
  private final Joystick joystick2 = new Joystick(1);

  public RobotContainer() {

    configureBindings();
    m_ElevatorSubsystem.setDefaultCommand(new ElevatorMovePid(m_ElevatorSubsystem, ElevatorConstantsPid.SetPoint));
  }

  public void configureBindings() {
    
    m_driverController.b().whileTrue(new ElevatorMoveUp(m_ElevatorSubsystem));
    m_driverController.a().whileTrue(new ElevatorMoveDown(m_ElevatorSubsystem));
    m_driverController.y().whileTrue(new ElevatorMovePid(m_ElevatorSubsystem, ElevatorConstantsPid.SetPoint));
    new DriveForward(m_DriveSubsystem, joystick1.getY() * DrivingConstants.SpeedTuning, joystick2.getX() * DrivingConstants.TurnTuning);
  }
}