package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.List;

public class teleop extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    Lift lift = new Lift();
    Motor intake;

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        lift.init(hardwareMap);
        intake=new Motor(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);


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

            lift.update();
        }
    }
}
