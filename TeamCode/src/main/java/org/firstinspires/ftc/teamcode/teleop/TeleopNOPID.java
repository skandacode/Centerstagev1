package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
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
public class TeleopNOPID extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    Lift lift=new Lift();
    Motor intake;
    Servo deposit;
    Servo hangservo;
    Servo droneservo;
    DcMotor hang;

    public static double intakespeed=0.5;

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


        waitForStart();
        while (opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);

            double heading = drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (gamepad1.right_bumper){
                drivetrain.driveRobotCentric(gamepad2.left_stick_x/3.0, gamepad2.left_stick_y/3.0, -0.23*gamepad2.right_stick_x);
            }
            else{
                drivetrain.driveRobotCentric(gamepad2.left_stick_x, gamepad2.left_stick_y, -0.7*gamepad2.right_stick_x);
            }
            if (gamepad2.a){
                intake.set(0.4);
            }else if (gamepad2.b) {
                intake.set(-intakespeed);
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
            if (gamepad2.dpad_down){
                hangservo.setPosition(1);//down
            }
            if (gamepad2.dpad_right){
                hangservo.setPosition(0);//up
            }
            if (gamepad2.dpad_up){
                hangservo.setPosition(0.25);//locks before hang
            }
            if (gamepad2.dpad_left){
                hangservo.setPosition(0.5);//drone
            }
            if (!gamepad2.y){
                droneservo.setPosition(0.9);//shoot drone
            }else{
                droneservo.setPosition(0);//keep drone
            }
            hang.setPower(gamepad1.right_trigger-gamepad1.left_trigger);//hang
        }
    }
}