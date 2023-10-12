package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@Config
@TeleOp
public class Localizer extends LinearOpMode {
    MecanumDrive drive;


    public static double TRACKWIDTH = 9.172;
    public static double TICKS_TO_INCHES = 0.000537549367;
    public static double CENTER_WHEEL_OFFSET = -5.944;

    MotorEx encoderLeft, encoderRight, encoderPerp;

    @Override
    public void runOpMode() throws InterruptedException {
        drive=new MecanumDrive(
                new Motor(hardwareMap, "frontleft"),
                new Motor(hardwareMap, "frontright"),
                new Motor(hardwareMap, "backleft"),
                new Motor(hardwareMap, "backright"));


        encoderLeft = new MotorEx(hardwareMap, "backleft");
        encoderRight = new MotorEx(hardwareMap, "backright");
        encoderPerp = new MotorEx(hardwareMap, "frontleft");

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


        waitForStart();

        localizer.updatePose(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

        while (opModeIsActive()){
            drive.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            localizer.updatePose();

            Pose2d position=localizer.getPose();
            telemetry.addLine(position.toString());
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();


            packet.fieldOverlay().setFill("blue")
                        .strokeCircle(position.getX(), position.getY(), 9)
                        .strokeLine(position.getX(), position.getY(),
                                (position.getRotation().getCos()*10)+ position.getX(),
                                (position.getRotation().getSin()*10)+ position.getY());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}
