package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
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
@Config
public class Red2plus2 extends LinearOpMode {

    Lift lift = new Lift();
    Motor intake;
    Servo intakeheights;
    public static double intake_heights_down=0.4;
    public static double intake_heights_up=0.015;
    public static double intakespeed=0.7;
    OpenCvWebcam webcam;
    public static String ObjectDirection;
    enum AutoStates{
        TOBACKBOARD,
        LIFT,
        HALFOPEN,
        FULLYOPEN,
        RETRACT,
        CYCLE,
        LIFT2,
        HALFOPEN2,
        FULLYOPEN2,
        RETRACT2,
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

        intakeheights=hardwareMap.servo.get("intakeheights");
        intake=new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        FtcDashboard dashboard= FtcDashboard.getInstance();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence rightpath = drive.trajectorySequenceBuilder(new Pose2d(10, -65.18, Math.toRadians(90)))
                .lineTo(new Vector2d(21, -40))
                .lineTo(new Vector2d(24, -54))
                .lineToLinearHeading(new Pose2d(50, -43, Math.toRadians(0)))
                .build();

        TrajectorySequence middlepath = drive.trajectorySequenceBuilder(new Pose2d(10, -65.18, Math.toRadians(90)))
                .lineTo(new Vector2d(10, -35))
                .lineTo(new Vector2d(20.70, -48.03))
                .lineToLinearHeading(new Pose2d(50, -38, Math.toRadians(0)))
                .build();

        TrajectorySequence leftpath = drive.trajectorySequenceBuilder(new Pose2d(10, -65.18, Math.toRadians(90)))
                .splineTo(new Vector2d(6, -37), Math.toRadians(360.0-194.93))
                .lineToLinearHeading(new Pose2d(31.45, -35.05, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(30.75, -35.10, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(50, -32, Math.toRadians(0)))
                .build();


        TrajectorySequence cycle = drive.trajectorySequenceBuilder(new Pose2d(40, -36, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(30, -15, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> {
                    intake.set(intakespeed);
                    intakeheights.setPosition(intake_heights_up);
                })
                .lineToLinearHeading(new Pose2d(-61, -15, Math.toRadians(0.00)))
                .lineToLinearHeading(new Pose2d(-58, -15, Math.toRadians(0.00)))
                .addTemporalMarker(() -> {
                    intakeheights.setPosition(intake_heights_down);
                    intake.set(intakespeed*0.7);
                })
                .waitSeconds(2)
                .UNSTABLE_addDisplacementMarkerOffset(2, () -> {
                    intake.set(-1);
                })
                .lineToLinearHeading(new Pose2d(30, -13, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> {
                    intake.set(0);
                })
                .lineToLinearHeading(new Pose2d(49, -36, Math.toRadians(2.00)))
                .build();



        drive.setPoseEstimate(rightpath.start());

        StateMachine leftmachine = new StateMachineBuilder()
                .state(Blue2plus2.AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(leftpath))
                .transition(() -> !drive.isBusy())
                .state(Blue2plus2.AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(475))
                .transition(() -> lift.getEncoderPos()>425)
                .state(Blue2plus2.AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(1)
                .state(Blue2plus2.AutoStates.FULLYOPEN)
                .onEnter(()->lift.open())
                .transitionTimed(1)
                .state(Blue2plus2.AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(0);
                    lift.close();
                }).transition(()->lift.is_down())
                .state(Blue2plus2.AutoStates.CYCLE)
                .onEnter(()-> {
                    drive.followTrajectorySequenceAsync(cycle);
                    lift.setPower(0);
                })
                .transition(()->!drive.isBusy())
                .state(Blue2plus2.AutoStates.LIFT2)
                .onEnter(() -> lift.setTarget(750))
                .transition(() -> lift.getEncoderPos()>650)
                .state(Blue2plus2.AutoStates.HALFOPEN2)
                .onEnter(() -> lift.half_open())
                .transitionTimed(0.5)
                .state(Blue2plus2.AutoStates.FULLYOPEN2)
                .onEnter(()->lift.open())
                .transitionTimed(2)
                .state(Blue2plus2.AutoStates.RETRACT2)
                .onEnter(()->{
                    lift.setTarget(0);
                    lift.close();
                })
                .build();
        StateMachine middlemachine = new StateMachineBuilder()
                .state(Blue2plus2.AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(middlepath))
                .transition(() -> !drive.isBusy())
                .state(Blue2plus2.AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(475))
                .transition(() -> lift.getEncoderPos()>425)
                .state(Blue2plus2.AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(1)
                .state(Blue2plus2.AutoStates.FULLYOPEN)
                .onEnter(()->lift.open())
                .transitionTimed(1)
                .state(Blue2plus2.AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(0);
                    lift.close();
                }).transition(()->lift.is_down())
                .state(Blue2plus2.AutoStates.CYCLE)
                .onEnter(()-> {
                    drive.followTrajectorySequenceAsync(cycle);
                    lift.setPower(0);
                })
                .transition(()->!drive.isBusy())
                .state(Blue2plus2.AutoStates.LIFT2)
                .onEnter(() -> lift.setTarget(750))
                .transition(() -> lift.getEncoderPos()>650)
                .state(Blue2plus2.AutoStates.HALFOPEN2)
                .onEnter(() -> lift.half_open())
                .transitionTimed(0.5)
                .state(Blue2plus2.AutoStates.FULLYOPEN2)
                .onEnter(()->lift.open())
                .transitionTimed(2)
                .state(Blue2plus2.AutoStates.RETRACT2)
                .onEnter(()->{
                    lift.setTarget(0);
                    lift.close();
                })
                .build();

        StateMachine rightmachine = new StateMachineBuilder()
                .state(Blue2plus2.AutoStates.TOBACKBOARD)
                .onEnter(() -> drive.followTrajectorySequenceAsync(rightpath))
                .transition(() -> !drive.isBusy())
                .state(Blue2plus2.AutoStates.LIFT)
                .onEnter(() -> lift.setTarget(475))
                .transition(() -> lift.getEncoderPos()>425)
                .state(Blue2plus2.AutoStates.HALFOPEN)
                .onEnter(() -> lift.half_open())
                .transitionTimed(1)
                .state(Blue2plus2.AutoStates.FULLYOPEN)
                .onEnter(()->lift.open())
                .transitionTimed(1)
                .state(Blue2plus2.AutoStates.RETRACT)
                .onEnter(()->{
                    lift.setTarget(0);
                    lift.close();
                }).transition(()->lift.is_down())
                .state(Blue2plus2.AutoStates.CYCLE)
                .onEnter(()-> {
                    drive.followTrajectorySequenceAsync(cycle);
                    lift.setPower(0);
                })
                .transition(()->!drive.isBusy())
                .state(Blue2plus2.AutoStates.LIFT2)
                .onEnter(() -> lift.setTarget(750))
                .transition(() -> lift.getEncoderPos()>650)
                .state(Blue2plus2.AutoStates.HALFOPEN2)
                .onEnter(() -> lift.half_open())
                .transitionTimed(0.5)
                .state(Blue2plus2.AutoStates.FULLYOPEN2)
                .onEnter(()->lift.open())
                .transitionTimed(2)
                .state(Blue2plus2.AutoStates.RETRACT2)
                .onEnter(()->{
                    lift.setTarget(0);
                    lift.close();
                })
                .build();




        drive.setPoseEstimate(rightpath.start());

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

        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener()
        {
            @Override
            public void onClose() {

            }
        });


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
