// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SwerveDrive {
    public final double L = 28;
    public final double W = 28;

    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;

    public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
    }

    public void drive (double x1, double y1, double x2) {
        double r = Math.sqrt ((L * L) + (W * W));
    
        double a = x1 - x2 * (L / r);
        double b = x1 + x2 * (L / r);
        double c = y1 - x2 * (W / r);
        double d = y1 + x2 * (W / r);
    
        double backRightSpeed = Math.sqrt ((a * a) + (d * d));
        double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
        double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
        double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));
    
        double backRightAngle = Math.atan2 (a, d) / Math.PI;
        double backLeftAngle = Math.atan2 (a, c) / Math.PI;
        double frontRightAngle = Math.atan2 (b, d) / Math.PI;
        double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

        backRight.drive (0, 0); // backRightAngle);
        backLeft.drive(0, 0); //backLeftAngle);
        frontRight.drive (0, 0); //frontRightAngle);
        frontLeft.drive (0, 0);// frontLeftAngle);

        SmartDashboard.putNumber("a", a);
        SmartDashboard.putNumber("b", b);
        SmartDashboard.putNumber("c", c);
        SmartDashboard.putNumber("d", d);

        SmartDashboard.putNumber("backRightAngle", backRightAngle);
        SmartDashboard.putNumber("backLeftAngle", backLeftAngle);
        SmartDashboard.putNumber("frontRightAngle", frontRightAngle);
        SmartDashboard.putNumber("frontLeftAngle", frontLeftAngle);

        SmartDashboard.putNumber("backRightSpeed", backRightSpeed);
        SmartDashboard.putNumber("backLeftSpeed", backLeftSpeed);
        SmartDashboard.putNumber("frontRightSpeed", frontRightSpeed);
        SmartDashboard.putNumber("frontLeftSpeed", frontLeftSpeed);
    }
}
