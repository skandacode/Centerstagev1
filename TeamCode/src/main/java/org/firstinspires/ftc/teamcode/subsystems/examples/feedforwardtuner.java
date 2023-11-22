package org.firstinspires.ftc.teamcode.subsystems.examples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp
@Config
public class feedforwardtuner extends LinearOpMode {
    Drivetrain drivetrain=new Drivetrain();
    public static double forwardPower=0;
    public static double strafePower=0;
    public static double turnPower=0;


    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            drivetrain.driveRobotCentric(strafePower, forwardPower, turnPower);
        }

    }
}
