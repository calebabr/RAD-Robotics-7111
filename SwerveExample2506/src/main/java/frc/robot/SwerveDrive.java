// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

/** Add your docs here. */
public class SwerveDrive {
    public final double L = 28;
    public final double W = 28;

    private WheelDrive backRight;
    private WheelDrive backLeft;
    private WheelDrive frontRight;
    private WheelDrive frontLeft;

    double a;
    double b;
    double c;
    double d;

    double frontRightSpeed; // 1
    double frontLeftSpeed; // 2
    double backRightSpeed; // 4 
    double backLeftSpeed; // 3

    double frontRightAngle; // 1
    double frontLeftAngle; // 2 
    double backRightAngle; // 4
    double backLeftAngle; // 3
    double temp;
   

    private AHRS gyro = new AHRS(Port.kMXP);
    private double ahrsAngle;


    public SwerveDrive (WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
        this.backRight = backRight;
        this.backLeft = backLeft;
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
    }

    public void init(){
        frontRight.zeroWheel(90);
        // frontLeft.zeroWheel(0);
        // backLeft.zeroWheel(0);
        // backRight.zeroWheel(0);
    }
    
    
    public void drive (double x1, double y1, double x2) {
        double magnitude = Math.sqrt((Math.pow(x1, 2)) + (Math.pow(y1,2)));
        if (magnitude >= 0.25) {
            a = Robot.applyDeadband(x1 - x2 * (L / 2), 0.01);
            b = Robot.applyDeadband(x1 + x2 * (L / 2), 0.01);
            c = Robot.applyDeadband(y1 - x2 * (W / 2), 0.01);
            d = Robot.applyDeadband(y1 + x2 * (W / 2), 0.01);
    
            frontRightSpeed = Math.sqrt ((b * b) + (c * c)); // 1
            frontLeftSpeed = Math.sqrt ((b * b) + (d * d)); // 2
            backRightSpeed = Math.sqrt ((a * a) + (c * c)); // 4 
            backLeftSpeed = Math.sqrt ((a * a) + (d * d)); // 3
    
            frontRightAngle = Math.atan2 (b, c) / Math.PI; // 1
            frontLeftAngle = Math.atan2 (b, d) / Math.PI; // 2 
            backRightAngle = Math.atan2 (a, c) / Math.PI; // 4
            backLeftAngle = Math.atan2 (a, d) / Math.PI; // 3
        }
        else{
            a = 0;
            b = 0;
            c = 0;
            d = 0;
        
            frontRightSpeed = 0; // 1
            frontLeftSpeed = 0; // 2
            backRightSpeed = 0; // 4 
            backLeftSpeed = 0; // 3
        
            frontRightAngle = 0; // 1
            frontLeftAngle = 0; // 2 
            backRightAngle = 0; // 4
            backLeftAngle = 0; // 3
        }
        
        backRight.drive (backRightSpeed, backRightAngle); // backRightAngle);
        backLeft.drive(backLeftSpeed, backLeftAngle); //backLeftAngle);
        frontRight.drive (frontRightSpeed, frontRightAngle); //frontRightAngle);
        frontLeft.drive (frontLeftSpeed, frontLeftAngle);// frontLeftAngle);

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

        SmartDashboard.putNumber("FR Abs Enc", frontRight.enc.getAbsolutePosition());
        SmartDashboard.putNumber("FL Abs Enc", frontLeft.enc.getAbsolutePosition());
        SmartDashboard.putNumber("Bl Abs Enc", backLeft.enc.getAbsolutePosition());
        SmartDashboard.putNumber("BR Abs Enc", backRight.enc.getAbsolutePosition());
    }
}
