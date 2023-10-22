package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

public class Drivetrain {
    MecanumDrive drive;
    PIDFController forward_controller;
    PIDFController strafe_controller;

    static double forwardP = 0.025;
    static double forwardD = 0;
    static double strafeP = 0.05;
    static double strafeD = 0;

    static double forwardCoef = -1;
    static double strafeCoef = 1;


    PIDFController heading_controller;
    static double headingP = 0.01;
    static double headingD = 0;
    static double headingCoef = 1;


    static double TRACKWIDTH = 12.46063;
    static double TICKS_TO_INCHES = 0.0005221053927409524;
    static double CENTER_WHEEL_OFFSET = -7.086614173228347;


    MotorEx encoderLeft, encoderRight, encoderPerp;
    public IMU imu;
    public HolonomicOdometry localizer;

    public void init(HardwareMap hardwareMap) {
        drive = new MecanumDrive(
                new Motor(hardwareMap, "frontleft"),
                new Motor(hardwareMap, "frontright"),
                new Motor(hardwareMap, "backleft"),
                new Motor(hardwareMap, "backright"));


        encoderLeft = new MotorEx(hardwareMap, "backleft");
        encoderRight = new MotorEx(hardwareMap, "frontright");
        encoderPerp = new MotorEx(hardwareMap, "frontleft");

        encoderLeft.setDistancePerPulse(-TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(-TICKS_TO_INCHES);         //REVERSES ALL
        encoderPerp.setDistancePerPulse(-TICKS_TO_INCHES);

        encoderLeft.resetEncoder();
        encoderRight.resetEncoder();
        encoderPerp.resetEncoder();


        localizer = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );



        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
        forward_controller = new PIDFController(forwardP, 0, forwardD, 0);
        strafe_controller = new PIDFController(strafeP, 0, strafeD, 0);
        heading_controller = new PIDFController(headingP, 0, headingD, 0);

        imu.resetYaw();

        localizer.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));


    }

    public void update(double imuangle) {
        localizer.updatePose();
        Pose2d position = localizer.getPose();


        double rot = position.getRotation().getDegrees();
        if (Math.abs(rot - heading_controller.getSetPoint()) > 180) {
            if (rot > heading_controller.getSetPoint()) {
                rot = rot - 360.0;
            } else {
                rot = rot + 360.0;
            }
        }
        double strafepower = strafe_controller.calculate(position.getY()) * strafeCoef;
        double forwardpower = forward_controller.calculate(position.getX()) * forwardCoef;
        double headingpower = heading_controller.calculate(rot) * headingCoef;
        drive.driveFieldCentric(
                    strafepower,
                    forwardpower,
                    headingpower,
                    imuangle);

    }

    public void setTargetPosition(double x_set, double y_set, double h_set) {
        forward_controller.setSetPoint(x_set);
        strafe_controller.setSetPoint(-y_set);
        heading_controller.setSetPoint(h_set);
    }
    public void driveFieldCentric(double strafeSpeed, double forwardSpeed, double turn, double heading){
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);
    }
    public void driveRobotCentric(double strafeSpeed, double forwardSpeed, double turn){
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turn);
    }

}
