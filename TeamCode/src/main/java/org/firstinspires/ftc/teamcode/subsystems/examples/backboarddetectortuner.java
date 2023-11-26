package org.firstinspires.ftc.teamcode.subsystems.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
@TeleOp
@Config
public class backboarddetectortuner extends LinearOpMode {
    public static double threshold=6;
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        RevColorSensorV3 backboarddetector=(RevColorSensorV3) hardwareMap.colorSensor.get("backboarddetector");

        waitForStart();
        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();
            double distance= backboarddetector.getDistance(DistanceUnit.CM);
            packet.put("distance", distance);
            packet.put("threshold", threshold);
            if (distance> threshold){
                packet.put("status", "not at backboard");
            }else{
                packet.put("status", "at backboard");
            }
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
