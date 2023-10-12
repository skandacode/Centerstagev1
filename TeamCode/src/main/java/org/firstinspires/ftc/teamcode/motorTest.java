package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp
@Config
public class motorTest extends LinearOpMode {
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
    DcMotor intake;
    DcMotor hang;
    DcMotor outtake1, outtake2;

    public static double intakePower=1;
    @Override
    public void runOpMode() throws InterruptedException {
        frontleft=hardwareMap.dcMotor.get("frontleft");
        frontright=hardwareMap.dcMotor.get("frontright");
        backleft=hardwareMap.dcMotor.get("backleft");
        backright=hardwareMap.dcMotor.get("backright");

        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);


        intake=hardwareMap.dcMotor.get("intake");
        hang=hardwareMap.dcMotor.get("hang");
        outtake1=hardwareMap.dcMotor.get("outtake1");
        outtake2=hardwareMap.dcMotor.get("outtake2");

        outtake2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                frontleft.setPower(1);
            }else{
                frontleft.setPower(0);
            }
            if (gamepad1.b){
                frontright.setPower(1);
            }else{
                frontright.setPower(0);
            }
            if (gamepad1.x){
                backleft.setPower(1);
            }else{
                backleft.setPower(0);
            }
            if (gamepad1.y){
                backright.setPower(1);
            }else{
                backright.setPower(0);
            }
            if (gamepad2.a){
                intake.setPower(0.6);
            }else{
                intake.setPower(0);
            }
            if (gamepad2.b){
                hang.setPower(1);
            }else{
                hang.setPower(0);
            }
            if (gamepad2.x){
                outtake1.setPower(1);
            }else{
                outtake1.setPower(0);
            }
            if (gamepad2.y){
                outtake2.setPower(1);
            }else{
                outtake2.setPower(0);
            }
            telemetry.addData("Power", intakePower);
            telemetry.addData("frontleft", frontleft.getCurrentPosition());
            telemetry.addData("frontright", frontright.getCurrentPosition());
            telemetry.addData("backleft", backleft.getCurrentPosition());
            telemetry.addData("backright", backright.getCurrentPosition());
            telemetry.update();
        }


    }
}
