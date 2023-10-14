// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants;
import frc.robot.Constants.DrivingConstants;
import frc.robot.Constants.ElevatorConstantsPid;
import frc.robot.commands.Autonomous1;
import frc.robot.commands.Autonomous2;
import frc.robot.commands.DriveForward;
import frc.robot.commands.ElevatorMoveDown;
import frc.robot.commands.ElevatorMovePid;
import frc.robot.commands.ElevatorMoveUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();;
  private final Command Auto1 = new Autonomous1(m_DriveSubsystem, 0, 0);
  private final Command Auto2 = new Autonomous2(m_DriveSubsystem, 0, 0,5);
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(2);
  private final Joystick joystick1 = new Joystick(0);
  private final Joystick joystick2 = new Joystick(1);
  

  public RobotContainer() {
    SmartDashboard.putData(m_chooser);
    m_chooser.setDefaultOption("Simon's Auto", Auto1);
    m_chooser.addOption("Alex's Auto", Auto2);
    SmartDashboard.putData("James's Auto choices", m_chooser);
    
    configureBindings();
    m_ElevatorSubsystem.setDefaultCommand(new ElevatorMovePid(m_ElevatorSubsystem, ElevatorConstantsPid.SetPoint));
    m_DriveSubsystem.setDefaultCommand(new DriveForward(m_DriveSubsystem, () -> joystick1.getY() * DrivingConstants.SpeedTuning, () -> joystick2.getX() * DrivingConstants.TurnTuning));
  }

  public void configureBindings() {
    
    m_driverController.b().whileTrue(new ElevatorMoveUp(m_ElevatorSubsystem));
    m_driverController.a().whileTrue(new ElevatorMoveDown(m_ElevatorSubsystem));
    m_driverController.y().whileTrue(new ElevatorMovePid(m_ElevatorSubsystem, ElevatorConstantsPid.SetPoint));
    
  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}