package org.firstinspires.ftc.teamcode.subsystems.examples;

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
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@Config
@TeleOp
public class PosePIDSExample extends LinearOpMode {
    MecanumDrive drive;
    public PIDFController forward_controller;
    public PIDFController strafe_controller;

    public static double forwardP=0.025;
    public static double forwardD=0;
    public static double strafeP=0.05;
    public static double strafeD=0;

    public static double forwardCoef=-1;
    public static double strafeCoef=1;


    public PIDFController heading_controller;
    public static double headingP=0.01;
    public static double headingD=0;
    public static double headingCoef=1;


    public static double TRACKWIDTH = 9.351882525;
    public static double TICKS_TO_INCHES = 0.0005221053927409524;
    public static double CENTER_WHEEL_OFFSET = -5.944;

    public static double x_set=0;
    public static double y_set=0;
    public static double h_set=0;


    MotorEx encoderLeft, encoderRight, encoderPerp;

    @Override
    public void runOpMode() throws InterruptedException {
        drive=new MecanumDrive(
                new Motor(hardwareMap, "frontleft"),
                new Motor(hardwareMap, "frontright"),
                new Motor(hardwareMap, "backleft"),
                new Motor(hardwareMap, "backright"));


        encoderLeft = new MotorEx(hardwareMap, "frontleft");
        encoderRight = new MotorEx(hardwareMap, "backright");
        encoderPerp = new MotorEx(hardwareMap, "frontright");

        encoderLeft.setDistancePerPulse(-TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(-TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(-TICKS_TO_INCHES);

        encoderLeft.resetEncoder();
        encoderRight.resetEncoder();
        encoderPerp.resetEncoder();


        HolonomicOdometry localizer = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        FtcDashboard dashboard= FtcDashboard.getInstance();

        IMU imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        forward_controller=new PIDFController(0, 0, 0, 0);
        strafe_controller=new PIDFController(0, 0, 0, 0);
        heading_controller=new PIDFController(0, 0, 0, 0);


        waitForStart();

        imu.resetYaw();

        localizer.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        while (opModeIsActive()){
            //drive.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            double imuangle=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            localizer.updatePose();

            Pose2d position=localizer.getPose();
            telemetry.addLine(position.toString());
            telemetry.addData("IMU", imuangle);
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();


            packet.fieldOverlay().setFill("blue")
                        .strokeCircle(position.getX(), position.getY(), 9)
                        .strokeLine(position.getX(), position.getY(),
                                (position.getRotation().getCos()*10)+ position.getX(),
                                (position.getRotation().getSin()*10)+ position.getY());

            dashboard.sendTelemetryPacket(packet);

            /*forward_controller.setP(forwardP);
            forward_controller.setD(forwardD);

            strafe_controller.setP(strafeP);
            strafe_controller.setD(strafeD);

            heading_controller.setP(headingP);
            heading_controller.setD(headingD);

            forward_controller.setSetPoint(x_set);
            strafe_controller.setSetPoint(y_set);
            heading_controller.setSetPoint(h_set);*/

            double rot= position.getRotation().getDegrees();
            if (Math.abs(rot-h_set)>180){
                if (rot>h_set){
                    rot=rot-360.0;
                }else{
                    rot=rot+360.0;
                }
            }
            double strafepower=strafe_controller.calculate(position.getY())*strafeCoef;
            double forwardpower=forward_controller.calculate(position.getX())*forwardCoef;
            double headingpower=heading_controller.calculate(rot)*headingCoef;

            if (strafepower+forwardpower+headingpower<0.02){
                drive.driveFieldCentric(0, 0, 0, imuangle);
            }else{
                drive.driveFieldCentric(
                        strafepower,
                        forwardpower,
                        headingpower,
                        imuangle);
            }
        }
    }
}
