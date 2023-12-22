package org.firstinspires.ftc.teamcode.debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class servotest extends LinearOpMode {
    Servo hangservo;
    Servo droneservo;
    DcMotor hang;
    Servo yellowpixel;
    Servo intakeheights;

    public static double hangpos=0;
    public static double yellowpos=0;
    public static double dronepos=0;
    public static double intakeheight=0;



    @Override
    public void runOpMode() throws InterruptedException {
        hangservo=hardwareMap.servo.get("deposit");
        droneservo=hardwareMap.servo.get("drone");
        droneservo=hardwareMap.servo.get("drone");
        yellowpixel=hardwareMap.servo.get("yellowpixel");
        hang=hardwareMap.dcMotor.get("hang");
        intakeheights=hardwareMap.servo.get("intakeheights");

        waitForStart();
        while (opModeIsActive()){
            hangservo.setPosition(hangpos);
            droneservo.setPosition(dronepos);
            hang.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            yellowpixel.setPosition(yellowpos);
            intakeheights.setPosition(intakeheight);
        }
    }
}
