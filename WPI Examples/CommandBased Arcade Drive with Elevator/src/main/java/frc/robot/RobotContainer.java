// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.ElevatorJoy;
import frc.robot.commands.AutoRoutine1;
import frc.robot.commands.AutoRoutine2;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.Command;
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
  
  private DriveSubsystem driveSub = new DriveSubsystem();
  private ElevatorSubsystem elevatorSub = new ElevatorSubsystem();
  private Joystick leftJoy = new Joystick(0);
  private Joystick rightJoy = new Joystick(1);
  private XboxController xbox = new XboxController(2);
  private CommandXboxController xboxController = new CommandXboxController(2);
  
  private final AutoRoutine1 straightAuto = new AutoRoutine1(driveSub, 5, 0.2);
  private final AutoRoutine2 spinAuto = new AutoRoutine2(driveSub, 10, 0.2, -0.2);
  SendableChooser<Command> chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  
    driveSub.setDefaultCommand(new DriveCommand(driveSub, () -> leftJoy.getY(), () -> rightJoy.getY()));
    chooser.setDefaultOption("Straight Auto", straightAuto);
    chooser.addOption("Spin Auto", spinAuto);
    SmartDashboard.putData(chooser);
    // elevatorSub.setDefaultCommand(new ElevatorJoy(elevatorSub, 0));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    xboxController.rightBumper().whileTrue(new ElevatorPID(elevatorSub, 100000));
    xboxController.leftBumper().whileTrue(new ElevatorPID(elevatorSub, 0));
    xboxController.a().whileTrue(new ElevatorJoy(elevatorSub, 0.5));
    xboxController.b().whileTrue(new ElevatorJoy(elevatorSub, -0.5));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return chooser.getSelected();
  }
}
