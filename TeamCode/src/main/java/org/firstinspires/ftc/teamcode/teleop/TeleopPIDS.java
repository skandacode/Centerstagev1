package org.firstinspires.ftc.teamcode.teleop;
//hi
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;
@TeleOp
@Config
public class TeleopPIDS extends LinearOpMode {
    Lift lift=new Lift();
    Motor intake;
    Servo deposit;
    Servo hangservo;
    Servo droneservo;
    DcMotor hang;
    Servo intakeheights;
    public static double intakespeed=0.65;
    public static double intake_heights_down=0.4;
    public static double intake_heights_up=0.02;
    public static int stepSize=150;

    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx frontleft=new MotorEx(hardwareMap, "frontleft");
        MotorEx frontright=new MotorEx(hardwareMap, "frontright");
        MotorEx backleft=new MotorEx(hardwareMap, "backleft");
        MotorEx backright=new MotorEx(hardwareMap, "backright");

        frontleft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        MecanumDrive drivetrain=new MecanumDrive(frontleft,frontright,backleft,backright);
        lift.init(hardwareMap);
        intake=new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        deposit = hardwareMap.get(Servo.class, "deposit");
        hangservo=hardwareMap.servo.get("hangservo");
        droneservo=hardwareMap.servo.get("drone");
        hang=hardwareMap.dcMotor.get("hang");
        intakeheights=hardwareMap.servo.get("intakeheights");

        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
        FtcDashboard dashboard= FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        GamepadEx game2 = new GamepadEx(gamepad2);
        int lift_height=0;

        TriggerReader rightTriggerReader = new TriggerReader(
                game2, GamepadKeys.Trigger.RIGHT_TRIGGER
        );
        TriggerReader leftTriggerReader = new TriggerReader(
                game2, GamepadKeys.Trigger.LEFT_TRIGGER
        );

        intakeheights.setPosition(intake_heights_down);

        while (opModeInInit()){
            lift.setPower(-0.1);
        }
        lift.resetEncoder();
        waitForStart();
        double loopTime=0.0;
        lift.resetEncoder();
        ElapsedTime timer=new ElapsedTime();
        boolean prev_open=false;

        while (opModeIsActive()) {
            hubs.forEach(LynxModule::clearBulkCache);
            leftTriggerReader.readValue();
            rightTriggerReader.readValue();
            //double heading = drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            if (gamepad1.right_bumper || gamepad2.a || gamepad2.right_trigger>0.5){
                drivetrain.driveRobotCentric(-gamepad1.left_stick_x/4.0, gamepad1.left_stick_y/4.0, -0.15*gamepad1.right_stick_x);
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

            if (rightTriggerReader.wasJustPressed()){
                lift_height=lift_height+stepSize;
                if (lift_height>1000){
                    lift_height=0;
                }

            }
            if (leftTriggerReader.wasJustPressed()){
                lift_height=lift_height-stepSize;
                if (lift_height<0){
                    lift_height=1000;
                }
            }
            if (gamepad2.touchpad){
                lift_height=0;
            }
            lift.setTarget(lift_height);
            lift.update();
            boolean current_open=gamepad2.right_bumper && !lift.is_down();
            if (current_open){
                if (!prev_open) {//first time it is opened
                    timer.reset();
                    lift.half_open_teleop();
                }else if (timer.milliseconds()<500){
                    lift.half_open_teleop();
                }else{
                    lift.open();
                }
            }else{
                lift.close();
            }
            prev_open=current_open;

            if (gamepad2.left_bumper){
                lift.extendYellow();
            }else{
                lift.retractYellow();
            }
            if (gamepad2.ps){
                intakeheights.setPosition(intake_heights_up);
            }else{
                intakeheights.setPosition(intake_heights_down);
            }
            if (gamepad2.dpad_down){hangservo.setPosition(1);}//down
            if (gamepad2.dpad_right){hangservo.setPosition(0);}//up
            if (gamepad2.dpad_up){hangservo.setPosition(0.25);}//locks before hang
            if (gamepad2.dpad_left){hangservo.setPosition(0.4);}//drone
            if (gamepad2.y){
                droneservo.setPosition(0);//shoot drone
            }else{
                droneservo.setPosition(0.6);//keep drone
            }
            hang.setPower(gamepad1.right_trigger-gamepad1.left_trigger);//hang


            double loop = System.nanoTime();
            telemetry.addData("hz ", 1000000000 / (loop - loopTime));
            loopTime = loop;
            telemetry.update();
            /*TelemetryPacket packet = new TelemetryPacket();
            packet.put("frontleft current", frontleft.motorEx.getCurrent(CurrentUnit.AMPS));
            packet.put("frontright current", frontright.motorEx.getCurrent(CurrentUnit.AMPS));
            packet.put("backleft current", backleft.motorEx.getCurrent(CurrentUnit.AMPS));
            packet.put("backright current", backright.motorEx.getCurrent(CurrentUnit.AMPS));
            dashboard.sendTelemetryPacket(packet);*/

        }
    }
}