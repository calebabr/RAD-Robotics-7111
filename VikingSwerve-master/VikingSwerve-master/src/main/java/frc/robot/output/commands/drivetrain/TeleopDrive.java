package frc.robot.output.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveWheelController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

public class TeleopDrive extends CommandBase {

	private SwerveWheelController swerve = null;

	private boolean currentFOD = false;
	private Joystick leftJoy = new Joystick(0);
	private Joystick rightJoy = new Joystick(1);
	private XboxController xbox = new XboxController(2);

	public TeleopDrive() {
		swerve = SwerveWheelController.getInstance();

		addRequirements(swerve);
	}

	@Override
	public void initialize() {
		currentFOD = swerve.getFOD();

		swerve.resetGyro();
	}

	@Override
	public void execute() {
		if (xbox.getAButtonPressed()) {
			swerve.resetGyro();
		}

		if (xbox.getBButtonPressed()) {
			currentFOD = !currentFOD;
			swerve.setFOD(currentFOD);
		}

		swerve.drive(leftJoy.getX(), leftJoy.getY(),
					 rightJoy.getX(), swerve.gyroAngle());
	}
}
