package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.vision.BluePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.*;
import java.lang.*;
import java.io.*;
@Autonomous
public class FarBlueAuto extends LinearOpMode {


    Drivetrain drive=new Drivetrain();
    Lift lift=new Lift();
    OpenCvWebcam webcam;
    Motor intake;
    public static String ObjectDirection;

    public static PropPosition randomization=PropPosition.NONE;


    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        BluePipeline pipeline = new BluePipeline(telemetry, ObjectDirection);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });


        drive.init(hardwareMap);
        lift.init(hardwareMap);

        FtcDashboard dashboard= FtcDashboard.getInstance();


        while (opModeInInit()){
            telemetry.addData("Frame Count", webcam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
            telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
            intake=new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
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
        ElapsedTime autotimer=new ElapsedTime();


        if (randomization==PropPosition.LEFT){

            drive.setTargetPosition(-28, -5, 00);
            while (opModeIsActive()&&autotimer.milliseconds()<1600){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-28, 5,90);
            while (opModeIsActive()&&autotimer.milliseconds()<2300){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-26, -10, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<4000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            intake.set(-0.5);
            drive.setTargetPosition(-28, -24, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<6000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            intake.set(0.55);
            drive.setTargetPosition(-28, -20, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<7000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-28, -24, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<7500){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-28, -20, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<8000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-28, -24, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<8500){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            intake.set(-0.5);
            drive.setTargetPosition(-15, -10, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<9000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            /*
            drive.setTargetPosition(-30, 41, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<6000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-30, 41, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<7000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            lift.setTarget(500);
            while (opModeIsActive()&&autotimer.milliseconds()<8000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
            }
            while (opModeIsActive()&&autotimer.milliseconds()<9000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.open();
            }
            while (opModeIsActive()&&autotimer.milliseconds()<11000) {
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.close();
            }
            while (opModeIsActive()&&autotimer.milliseconds()<12000) {
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.setTarget(-5);
            }
            while (opModeIsActive()&&autotimer.milliseconds()<13000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
            }*/
        }else if (randomization==PropPosition.MIDDLE) { // --------------------------------------------------------------

            drive.setTargetPosition(-29.5, 0, 0);
            while (opModeIsActive()&&autotimer.milliseconds()<1600){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-10, 0, 0);
            while (opModeIsActive()&&autotimer.milliseconds()<3000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-28, 42, 85);
            while (opModeIsActive()&&autotimer.milliseconds()<6000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            lift.setTarget(500);
            while (opModeIsActive()&&autotimer.milliseconds()<8000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
            }
            while (opModeIsActive()&&autotimer.milliseconds()<10000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.open();
            }
            while (opModeIsActive()&&autotimer.milliseconds()<12000) {
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.close();
            }
            while (opModeIsActive()&&autotimer.milliseconds()<12500) {
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.setTarget(-5);
            }
            while (opModeIsActive()&&autotimer.milliseconds()<13000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
            }

        } else if (randomization==PropPosition.RIGHT) { // -============================================================
            drive.setTargetPosition(-28, 5, 00);
            while (opModeIsActive()&&autotimer.milliseconds()<1600){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-28, -5,-90);
            while (opModeIsActive()&&autotimer.milliseconds()<2300){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-30, 20, -90);
            while (opModeIsActive()&&autotimer.milliseconds()<4000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-30, 20, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<5000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-30, 41, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<6000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            drive.setTargetPosition(-30, 41, 90);
            while (opModeIsActive()&&autotimer.milliseconds()<7000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                drive.update(imuangle);
            }
            lift.setTarget(500);
            while (opModeIsActive()&&autotimer.milliseconds()<8000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
            }
            while (opModeIsActive()&&autotimer.milliseconds()<9000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.open();
            }
            while (opModeIsActive()&&autotimer.milliseconds()<11000) {
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.close();
            }
            while (opModeIsActive()&&autotimer.milliseconds()<12000) {
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
                lift.setTarget(-5);
            }
            while (opModeIsActive()&&autotimer.milliseconds()<13000){
                double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                lift.update();
                drive.update(imuangle);
            }
        }else{//Not found

        }


    }
}