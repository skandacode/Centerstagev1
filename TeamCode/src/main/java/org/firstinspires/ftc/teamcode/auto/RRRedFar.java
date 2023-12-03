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
import org.firstinspires.ftc.teamcode.vision.RedPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.Objects;
@Autonomous
public class RRRedFar extends LinearOpMode {

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

        TrajectorySequence rightpath = drive.trajectorySequenceBuilder(new Pose2d(-34, -67, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-48.93, -34.21, Math.toRadians(0.00)))
                .forward(18)
                .lineToLinearHeading(new Pose2d(-53, -34, Math.toRadians(0.00)))
                .lineToSplineHeading(new Pose2d(-52, -15, Math.toRadians(-7.00)))
                .lineToLinearHeading(new Pose2d(20, -25, Math.toRadians(-10.00)))
                .lineToLinearHeading(new Pose2d(43, -25, Math.toRadians(-9.00)))
                .lineToLinearHeading(new Pose2d(43, -65, Math.toRadians(0.00)))
                .lineToLinearHeading(new Pose2d(55, -65, Math.toRadians(0)))
                .build();



        TrajectorySequence middlepath = drive.trajectorySequenceBuilder(new Pose2d(-34, -67, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-37, -37, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-36, -49, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-53, -50, Math.toRadians(90.00)))
                .turn(Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-45, -15, Math.toRadians(-7.00)))
                .lineToLinearHeading(new Pose2d(35, -30, Math.toRadians(-7.00)))
                .lineToLinearHeading(new Pose2d(35, -59, Math.toRadians(-7.00)))
                .lineToLinearHeading(new Pose2d(55, -59, Math.toRadians(-7.00)))
                .build();


        TrajectorySequence leftpath = drive.trajectorySequenceBuilder(new Pose2d(-34, -67, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-46, -45, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-33, -60, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-36, -20, Math.toRadians(90.00)))
                .turn(Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(-52, -15, Math.toRadians(-3.00)))
                .lineToLinearHeading(new Pose2d(20, -15, Math.toRadians(-3.00)))
                .lineToLinearHeading(new Pose2d(35, -30, Math.toRadians(-5.00)))
                .lineToLinearHeading(new Pose2d(35, -38, Math.toRadians(-5.00)))
                .lineToLinearHeading(new Pose2d(55, -38, Math.toRadians(0)))
                .build();

        drive.setPoseEstimate(leftpath.start());



        StateMachine leftmachine = new StateMachineBuilder()
                .state(AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(leftpath))
                .transition(() -> !drive.isBusy())
                .state(AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(500))
                .transition(() -> lift.getEncoderPos()>400)
                .state(AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(2)
                .state(AutoStates.FULLYOPEN)
                .onEnter(()->lift.open())
                .transitionTimed(2)
                .state(AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(-50);
                    lift.close();
                })
                .build();
        StateMachine middlemachine = new StateMachineBuilder()
                .state(AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(middlepath))
                .transition(() -> !drive.isBusy())
                .state(AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(500))
                .transition(() -> lift.getEncoderPos()>400)
                .state(AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(2)
                .state(AutoStates.FULLYOPEN)
                .onEnter(()->lift.open())
                .transitionTimed(2)
                .state(AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(-50);
                    lift.close();
                })
                .build();
        StateMachine rightmachine = new StateMachineBuilder()
                .state(AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(rightpath))
                .transition(() -> !drive.isBusy())
                .state(AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(500))
                .transition(() -> lift.getEncoderPos()>400)
                .state(AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(2)
                .state(AutoStates.FULLYOPEN)
                .onEnter(()->lift.open())
                .transitionTimed(2)
                .state(AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(-50);
                    lift.close();
                })
                .build();


        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        lift.close();

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
