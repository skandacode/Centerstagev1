package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
public class RRBlueClose extends LinearOpMode {

    Lift lift=new Lift();
    OpenCvWebcam webcam;
    public static String ObjectDirection;

    public static PropPosition randomization=PropPosition.NONE;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        BluePipeline pipeline = new BluePipeline(telemetry, ObjectDirection);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {@Override
            public void onOpened(){webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);}
            @Override
            public void onError(int errorCode)
            {}});

        lift.init(hardwareMap);

        FtcDashboard dashboard= FtcDashboard.getInstance();

        SampleMecanumDrive drive=new SampleMecanumDrive(hardwareMap);

        TrajectorySequence leftpath = drive.trajectorySequenceBuilder(new Pose2d(9.85, 65.18, Math.toRadians(270.00)))
                .lineTo(new Vector2d(22.88, 35.75))
                .lineTo(new Vector2d(24.07, 53.98))
                .lineToLinearHeading(new Pose2d(44.86, 36.15, Math.toRadians(2.90)))
                .splineTo(new Vector2d(55.16, 35.67), Math.toRadians(0.00))
                .lineToLinearHeading(new Pose2d(55.35, 41.3, Math.toRadians(0.00)))
                .build();

        TrajectorySequence middlepath = drive.trajectorySequenceBuilder(new Pose2d(9.85, 65.18, Math.toRadians(270.00)))
                .lineTo(new Vector2d(10.37, 31.31))
                .lineTo(new Vector2d(20.70, 48.03))
                .lineToLinearHeading(new Pose2d(44.86, 36.15, Math.toRadians(2.90)))
                .splineTo(new Vector2d(55.16, 35.67), Math.toRadians(0.00))
                .lineToLinearHeading(new Pose2d(55.35, 36, Math.toRadians(0.00)))
                .build();

        TrajectorySequence rightpath = drive.trajectorySequenceBuilder(new Pose2d(9.85, 65.18, Math.toRadians(270.00)))
                .splineTo(new Vector2d(5.49, 33.92), Math.toRadians(194.93))
                .lineToLinearHeading(new Pose2d(31.45, 35.05, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(30.75, 35.10, Math.toRadians(2.90)))
                .splineTo(new Vector2d(55.16, 35.67), Math.toRadians(0.00))
                .lineToLinearHeading(new Pose2d(55.35, 30, Math.toRadians(0.00)))
                .build();


        drive.setPoseEstimate(rightpath.start());


        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));


        while (opModeInInit()){
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

            telemetry.update();

            if (Objects.equals(pipeline.getPosition(), "LEFT")) {
                telemetry.addData("Position", "LEFTpo");
                randomization=PropPosition.LEFT;
            }
            else if (Objects.equals(pipeline.getPosition(), "MIDDLE")){
                telemetry.addData("Position", "MIDDLEE");
                randomization=PropPosition.MIDDLE;
            }
            else if (Objects.equals(pipeline.getPosition(), "RIGHT")){
                telemetry.addData("Position", "RIGHTO");
                randomization=PropPosition.RIGHT;
            }

            sleep(100);
        }

        if (randomization==PropPosition.LEFT){
            drive.followTrajectorySequence(leftpath);
        }else if (randomization==PropPosition.MIDDLE){
            drive.followTrajectorySequence(middlepath);
        }else if (randomization==PropPosition.RIGHT){
            drive.followTrajectorySequence(rightpath);
        }
    }
}
