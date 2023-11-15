// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;


import edu.wpi.first.math.controller.PIDController;
/** Add your docs here. */
public class WheelDrive {
    private CANSparkMax steerMotor;
    private CANSparkMax driveMotor;
    private PIDController pidController;
    public CANCoder enc;
    private final double MAX_VOLTS = 0;
    private int quad;


    public WheelDrive (int angleMotor, int speedMotor, int encoder, int quad) {
        this.steerMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
        this.driveMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);
        this.enc = new CANCoder(encoder);
        this.quad = quad;
        pidController = new PIDController(0.002, 0.0002, 0.00002);

        pidController.enableContinuousInput(-180, 180);
        pidController.setSetpoint(0);
    }

    public void drive (double drive, double angle) {
        driveMotor.set(drive);
        steerMotor.set(angle);
    
        double setpoint = angle * (MAX_VOLTS * 0.25) + (MAX_VOLTS * 0.25); // Optimization offset can be calculated here.
        if (setpoint < 0) {
            setpoint = MAX_VOLTS + setpoint;
        }
        if (setpoint > MAX_VOLTS) {
            setpoint = setpoint - MAX_VOLTS;
        }
    
        pidController.setSetpoint(setpoint);
        SmartDashboard.putNumber("SetPoint", setpoint);
        SmartDashboard.putNumber("Drive", drive);
        SmartDashboard.putNumber("Steer", angle);
    }

    public void zeroWheel(double goal){
        steerMotor.set(pidController.calculate(enc.getAbsolutePosition(), goal));
    }
}
