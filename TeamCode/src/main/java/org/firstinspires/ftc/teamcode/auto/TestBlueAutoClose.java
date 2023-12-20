package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.BluePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;
@Autonomous
public class TestBlueAutoClose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard= FtcDashboard.getInstance();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence path = drive.trajectorySequenceBuilder(new Pose2d(10.04, 65.16, Math.toRadians(270.00)))
                .lineTo(new Vector2d(23.50, 30.77))
                .lineTo(new Vector2d(31.83, 57.26))
                .lineToLinearHeading(new Pose2d(50.51, 43.20, Math.toRadians(180.00)))
                .lineTo(new Vector2d(43.62, 9.92))
                .lineTo(new Vector2d(-61.10, 10.68))
                .lineTo(new Vector2d(32.91, 9.83))
                .lineTo(new Vector2d(50.51, 35.20))
                .build();



        drive.setPoseEstimate(path.start());

        waitForStart();

        drive.followTrajectorySequence(path);
    }
}
