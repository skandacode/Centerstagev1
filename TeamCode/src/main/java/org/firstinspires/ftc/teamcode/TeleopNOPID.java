package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;
@TeleOp
@Config
public class TeleopNOPID extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    Lift lift=new Lift();
    Motor intake;
    Servo deposit;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        lift.init(hardwareMap);
        intake=new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        deposit = hardwareMap.get(Servo.class, "deposit");


        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));


        waitForStart();
        while (opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            double heading = drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            drivetrain.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            if (gamepad2.a){
                intake.set(0.45);
            }else if (gamepad2.b) {
                intake.set(-0.45);
            }else{
                intake.set(0);
            }
            lift.setPower(-0.25*gamepad2.left_trigger+0.5*gamepad2.right_trigger);

            if (gamepad2.right_bumper){
                lift.open();
            }
            else{
                lift.close();
            }
        }
    }
}