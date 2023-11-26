package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class movewithapriltag extends LinearOpMode {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();

        waitForStart();
        double x=0;
        double y=0;
        double yaw=0;
        while (opModeIsActive() && !gamepad1.a){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            if (currentDetections.size()>0) {
                for (AprilTagDetection detection : currentDetections) {
                    if (detection.metadata != null) {
                        if (detection.id == 1) {
                            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                            x=detection.ftcPose.x;
                            y=detection.ftcPose.y;
                            yaw=detection.ftcPose.yaw;

                        }
                    }
                }
            }
            telemetry.update();
        }
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TrajectorySequence path = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .lineToLinearHeading(new Pose2d(y-7, -x, Math.toRadians(yaw)))
                .build();
        drive.followTrajectorySequence(path);
    }
}
