package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp
@Disabled
public class MoveToPOS extends LinearOpMode {
    Drivetrain drive=new Drivetrain();

    @Override
    public void runOpMode() throws InterruptedException {
        drive.init(hardwareMap);
        FtcDashboard dashboard= FtcDashboard.getInstance();

        waitForStart();
        while (opModeIsActive()){
            double imuangle = drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            if (gamepad1.y){
                drive.setTargetPosition(-20, 10, 0);
            }
            if (gamepad1.b){
                drive.setTargetPosition(-30, 35, 90);
            }
            TelemetryPacket packet = new TelemetryPacket();

            Pose2d position = drive.localizer.getPose();

            packet.fieldOverlay().setFill("blue")
                    .strokeCircle(position.getX(), position.getY(), 9)
                    .strokeLine(position.getX(), position.getY(),
                            (position.getRotation().getCos()*10)+ position.getX(),
                            (position.getRotation().getSin()*10)+ position.getY());

            dashboard.sendTelemetryPacket(packet);
            drive.update(imuangle);
        }
    }
}
