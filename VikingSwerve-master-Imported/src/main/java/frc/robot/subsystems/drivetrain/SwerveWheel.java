package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import viking.controllers.SwerveWheelDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;

public class SwerveWheel extends PIDSubsystem implements SwerveDrivetrainConstants {

	public String name;

	private CANSparkMax steerMotor;
	private AnalogInput absEnc;
	private CANSparkMax drive;
	private CANCoder steerEnc;

	private int countsWhenFrwd;

	public SwerveWheel(CANSparkMax drive, int m_steer, int enc, int zeroOffset,
					   String name) {
		super(new PIDController(kP, kI, kD));

		this.name = name;

		this.drive = drive;

		countsWhenFrwd = zeroOffset;

		steerMotor = new CANSparkMax(m_steer, MotorType.kBrushless);
		steerEnc = new CANCoder(enc);

		// Reset all of the settings on startup
		// steerMotor.configFactoryDefault();

		// Set the feedback device for the steering (turning) Talon SRX
		// steerMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

		// Set the current quadrature position relative to the analog position to make sure motor
		// has 0 position on startup
		// steerMotor.setSelectedSensorPosition(getAbsAngleDeg() * QUAD_COUNTS_PER_ROT / 180);

		// Set the input range of the PIDF so that it will only accept angles between -180 to 180
		// and set it to continuous
		getController().enableContinuousInput(0, 360);

		// Sets name for viewing in SmartDashboard
		this.setName(name);
	}

	/** 
	// Get the current angle of the analog encoder
	private int getAbsAngleDeg() {
		return (int)(180 * (absEnc.getValue() - countsWhenFrwd) / 4096);
	}

	// Get current ticks
	public int getTicks() {
		return (int)steerMotor.getSelectedSensorPosition();
	}

	// Convert ticks to angle bound from -180 to 180
	public double ticksToAngle(int ticks) {
		double angleTicks = ticks % QUAD_COUNTS_PER_ROT;

		double result = (angleTicks / (QUAD_COUNTS_PER_ROT / 2)) * 180;

		if (result > 180) {
			result -= 360;
		}

		return result;
	}
	*/

	public void setSpeed(double speed) {
		drive.set(speed);
	}

	@Override
	protected double getMeasurement() {
		return steerEnc.getAbsolutePosition();
	}

	@Override
	protected void useOutput(double output, double setpoint) {
		steerMotor.set(output);
	}
}
