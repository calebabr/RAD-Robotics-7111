package frc.robot.subsystems.drivetrain;

public interface SwerveDrivetrainConstants {

	// Length and Width of robot in inches
	public final double L = 28;
	public final double W = 28; // already correct

	// PIDF Variables
	public final double kP = 0.02;
	public final double kI = 0.0;
	public final double kD = 0.0;
	public final double kF = 0.0; // need to tune
	
	// Quadrature Encoder Ticks per Rotation
	public final int QUAD_COUNTS_PER_ROT = 1658; // need to figure out

	// Talon SRX Turn Motor CAN ID (change to sparkmax)
	public final int frontLeftTurnID = 4;
	public final int frontRightTurnID = 6;
	public final int backLeftTurnID = 2;
	public final int backRightTurnID = 8;

	// IDs for Drive Motors (change to sparkmax)
	public final int frontLeftDriveID = 3;
	public final int frontRightDriveID = 5;
	public final int backLeftDriveID = 1;
	public final int backRightDriveID = 7;

	// Analog Encoder ID (change to CANcoder, not analog)
	public final int frontLeftEncoderID = 2;
	public final int frontRightEncoderID = 3;
	public final int backLeftEncoderID = 1;
	public final int backRightEncoderID = 4;

	// Offset of analog to make encoders face forward
	public final int frontLeftEncoderOffset = 1850;
	public final int frontRightEncoderOffset = 3675;
	public final int backLeftEncoderOffset = 550;
	public final int backRightEncoderOffset = 1600;
}