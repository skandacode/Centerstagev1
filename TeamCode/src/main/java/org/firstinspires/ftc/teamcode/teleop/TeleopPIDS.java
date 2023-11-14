package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;
@TeleOp
@Config
public class TeleopPIDS extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    Lift lift=new Lift();
    Motor intake;
    Servo deposit;
    Servo hangservo;
    Servo droneservo;
    DcMotor hang;

    public static double intakespeed=0.55;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        lift.init(hardwareMap);
        intake=new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        deposit = hardwareMap.get(Servo.class, "deposit");
        hangservo=hardwareMap.servo.get("hangservo");
        droneservo=hardwareMap.servo.get("drone");
        hang=hardwareMap.dcMotor.get("hang");



        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        FtcDashboard dashboard= FtcDashboard.getInstance();

        while (opModeInInit()){
            lift.setPower(-0.1);
        }

        waitForStart();
        double loopTime=0.0;
        lift.resetEncoder();

        while (opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            //double heading = drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (gamepad1.right_bumper){
                drivetrain.driveRobotCentric(-gamepad1.left_stick_x/5.0, gamepad1.left_stick_y/5.0, -0.15*gamepad1.right_stick_x);
            }
            else{
                drivetrain.driveRobotCentric(-gamepad1.left_stick_x, gamepad1.left_stick_y, -0.5*gamepad1.right_stick_x);
            }
            if (gamepad2.a){
                intake.set(intakespeed);
            }else if (gamepad2.b) {
                intake.set(-intakespeed);
            }else{
                intake.set(0);
            }

            if (gamepad2.right_trigger>0.3){
                lift.setTarget(1000);
                lift.update();
                telemetry.addLine("going up with pids");
            }else{
                if (lift.is_down()){
                        lift.setPower(0);
                        telemetry.addLine("all the way down");
                }else{
                        lift.setPower(-0.5);
                        telemetry.addLine("going down");
                }
            }

            if (gamepad2.right_bumper){lift.open();}else{lift.close();}
            if (gamepad2.dpad_down){hangservo.setPosition(1);}//down
            if (gamepad2.dpad_right){hangservo.setPosition(0);}//up
            if (gamepad2.dpad_up){hangservo.setPosition(0.25);}//locks before hang
            if (gamepad2.dpad_left){hangservo.setPosition(0.5);}//drone
            if (!gamepad2.y){
                droneservo.setPosition(0.9);//shoot drone
            }else{
                droneservo.setPosition(0);//keep drone
            }
            hang.setPower(gamepad1.right_trigger-gamepad1.left_trigger);//hang


            drivetrain.localizer.updatePose();

            TelemetryPacket packet = new TelemetryPacket();
            Pose2d position = drivetrain.localizer.getPose();
            packet.fieldOverlay().setFill("blue")
                    .strokeCircle(position.getX(), position.getY(), 9)
                    .strokeLine(position.getX(), position.getY(),
                            (position.getRotation().getCos()*10)+ position.getX(),
                            (position.getRotation().getSin()*10)+ position.getY());

            dashboard.sendTelemetryPacket(packet);

            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();


        }
    }
}