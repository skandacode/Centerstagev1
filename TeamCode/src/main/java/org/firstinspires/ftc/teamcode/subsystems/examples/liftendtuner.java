package org.firstinspires.ftc.teamcode.subsystems.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Lift;
@TeleOp
@Disabled
public class liftendtuner extends LinearOpMode {
    Lift lift=new Lift();

    @Override
    public void runOpMode() throws InterruptedException {
        lift.init(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();
        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();
            double distance= lift.getDistance();
            packet.put("distance", distance);
            packet.put("threshold", lift.down_cutoff);
            if (distance> lift.down_cutoff){
                packet.put("status", "up");
            }else{
                packet.put("status", "down");
            }
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
