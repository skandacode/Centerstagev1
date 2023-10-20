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
public class teleop extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    Lift lift = new Lift();
    Motor intake;
    Servo deposit;
    Motor outtake1;
    Motor outtake2;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        lift.init(hardwareMap);
        intake=new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        outtake1=new Motor(hardwareMap, "outtake1", Motor.GoBILDA.RPM_1150);
        outtake2=new Motor(hardwareMap, "outtake2", Motor.GoBILDA.RPM_1150);
        deposit = hardwareMap.get(Servo.class, "deposit");


        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));


        waitForStart();
        while (opModeIsActive()){

            hubs.forEach(LynxModule::clearBulkCache);

            double heading=drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            drivetrain.driveFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, heading);
            if (gamepad1.a){lift.setTarget(0);}
            if (gamepad1.b){lift.setTarget(100);}
            if (gamepad1.x){lift.setTarget(200);}
            if (gamepad1.y){lift.setTarget(400);}
            intake.set(gamepad1.left_stick_x*0.45);
            outtake1.set(0.5*(gamepad1.left_trigger-gamepad1.right_trigger));
            outtake2.set(0.5*(gamepad1.left_trigger-gamepad1.right_trigger));

            if (gamepad1.right_bumper){
                deposit.setPosition(0.5);

            }
            else{
                deposit.setPosition(1);
            }


            lift.update();
        }
    }
}
