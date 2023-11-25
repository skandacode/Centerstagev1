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
import org.firstinspires.ftc.teamcode.vision.RedPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;
@Autonomous
public class RRRedClose extends LinearOpMode {

    Lift lift = new Lift();
    OpenCvWebcam webcam;
    public static String ObjectDirection;

    enum AutoStates{
        TOBACKBOARD,
        LIFT,
        HALFOPEN,
        FULLYOPEN,
        RETRACT,
        PARK
    }

    public static PropPosition randomization=PropPosition.NONE;
    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName()
        );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(
                WebcamName.class, "Webcam 1"), cameraMonitorViewId
        );

        RedPipeline pipeline = new RedPipeline(telemetry, ObjectDirection);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened(){webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);}
            @Override
            public void onError(int errorCode)
            {}
        });

        lift.init(hardwareMap);

        FtcDashboard dashboard= FtcDashboard.getInstance();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence rightpath = drive.trajectorySequenceBuilder(new Pose2d(10, -65.18, Math.toRadians(90)))
                .lineTo(new Vector2d(21, -40))
                .lineTo(new Vector2d(24, -54))
                .lineToLinearHeading(new Pose2d(50, -44, Math.toRadians(0)))
                .build();

        TrajectorySequence middlepath = drive.trajectorySequenceBuilder(new Pose2d(10, -65.18, Math.toRadians(90)))
                .lineTo(new Vector2d(10, -35))
                .lineTo(new Vector2d(20.70, -48.03))
                .lineToLinearHeading(new Pose2d(50, -36, Math.toRadians(0)))
                .build();

        TrajectorySequence leftpath = drive.trajectorySequenceBuilder(new Pose2d(10, -65.18, Math.toRadians(90)))
                .splineTo(new Vector2d(6, -37), Math.toRadians(360.0-194.93))
                .lineToLinearHeading(new Pose2d(31.45, -35.05, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(30.75, -35.10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50, -29, Math.toRadians(0)))
                .build();

        TrajectorySequence rightpark = drive.trajectorySequenceBuilder(rightpath.end())
                .lineToLinearHeading(new Pose2d(30, -57, Math.toRadians(0.00)))
                .lineTo(new Vector2d(60, -62))
                .build();

        TrajectorySequence middlepark = drive.trajectorySequenceBuilder(middlepath.end())
                .lineToLinearHeading(new Pose2d(30, -57, Math.toRadians(0.00)))
                .lineTo(new Vector2d(60, -62))
                .build();
        TrajectorySequence leftpark = drive.trajectorySequenceBuilder(leftpath.end())
                .lineToLinearHeading(new Pose2d(30, -57, Math.toRadians(0.00)))
                .lineTo(new Vector2d(60, -62))
                .build();



        drive.setPoseEstimate(rightpath.start());

        StateMachine leftmachine = new StateMachineBuilder()
                .state(AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(leftpath))
                .transition(() -> !drive.isBusy())
                .state(AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(450))
                .transition(() -> lift.getEncoderPos()>400)
                .state(AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(2)
                .state(AutoStates.FULLYOPEN)
                .onEnter(()->lift.close())
                .transitionTimed(2)
                .state(AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(-50);
                    lift.close();
                }).transition(()->lift.is_down())
                .state(AutoStates.PARK)
                .onEnter(()->{
                    drive.followTrajectorySequenceAsync(leftpark);
                    lift.setTarget(0);
                })
                .build();
        StateMachine middlemachine = new StateMachineBuilder()
                .state(AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(middlepath))
                .transition(() -> !drive.isBusy())
                .state(AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(450))
                .transition(() -> lift.getEncoderPos()>400)
                .state(AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(2)
                .state(AutoStates.FULLYOPEN)
                .onEnter(()->lift.close())
                .transitionTimed(2)
                .state(AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(-50);
                    lift.close();
                }).transition(()->lift.is_down())
                .state(AutoStates.PARK)
                .onEnter(()->{
                    drive.followTrajectorySequenceAsync(middlepark);
                    lift.setTarget(0);
                })
                .build();
        StateMachine rightmachine = new StateMachineBuilder()
                .state(AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(rightpath))
                .transition(() -> !drive.isBusy())
                .state(AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(450))
                .transition(() -> lift.getEncoderPos()>400)
                .state(AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(2)
                .state(AutoStates.FULLYOPEN)
                .onEnter(()->lift.close())
                .transitionTimed(2)
                .state(AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(-50);
                    lift.close();
                }).transition(()->lift.is_down())
                .state(AutoStates.PARK)
                .onEnter(()->{
                    drive.followTrajectorySequenceAsync(rightpark);
                    lift.setTarget(0);
                })
                .build();


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
        waitForStart();

        webcam.closeCameraDevice();

        double loopTime=0.0;

        if (randomization==PropPosition.LEFT){
            leftmachine.start();
            while (opModeIsActive()){
                hubs.forEach(LynxModule::clearBulkCache);
                leftmachine.update();
                lift.update();
                drive.update();
                telemetry.addData("Height", lift.getEncoderPos());
                telemetry.addData("Target", lift.getSetPoint());
                double loop = System.nanoTime();
                telemetry.addData("hz ", 1000000000 / (loop - loopTime));
                loopTime = loop;
                telemetry.update();
            }
        }else if (randomization==PropPosition.MIDDLE){
            middlemachine.start();
            while (opModeIsActive()){
                hubs.forEach(LynxModule::clearBulkCache);
                middlemachine.update();
                lift.update();
                drive.update();
                telemetry.addData("Height", lift.getEncoderPos());
                telemetry.addData("Target", lift.getSetPoint());
                double loop = System.nanoTime();
                telemetry.addData("hz ", 1000000000 / (loop - loopTime));
                loopTime = loop;
                telemetry.update();
            }
        }else if (randomization==PropPosition.RIGHT){
            rightmachine.start();
            while (opModeIsActive()){
                hubs.forEach(LynxModule::clearBulkCache);
                rightmachine.update();
                lift.update();
                drive.update();
                telemetry.addData("Height", lift.getEncoderPos());
                telemetry.addData("Target", lift.getSetPoint());
                double loop = System.nanoTime();
                telemetry.addData("hz ", 1000000000 / (loop - loopTime));
                loopTime = loop;
                telemetry.update();
            }
        }
    }
}