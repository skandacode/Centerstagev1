package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Lift {
    private static MotorGroup liftMotors;
    Motor lift1;
    Motor lift2;
    Servo deposit;
    RevColorSensorV3 liftend;

    public static double kP=0.002;
    public static double kD=0;
    public static double kF=0.11;

    public static double down_cutoff=8;
    PIDFController liftController=new PIDFController(kP, 0, kD, 0);

    public void init(HardwareMap hardwareMap){
        lift1=new Motor(hardwareMap, "outtake1", Motor.GoBILDA.RPM_1150);
        lift2=new Motor(hardwareMap, "outtake2", Motor.GoBILDA.RPM_1150);
        deposit=hardwareMap.servo.get("deposit");
        liftend=(RevColorSensorV3) hardwareMap.colorSensor.get("liftend");

        lift2.setInverted(true);

        liftMotors=new MotorGroup(lift1, lift2);
        liftMotors.resetEncoder();
    }
    public void setTarget(int setPoint){
        liftController.setSetPoint(setPoint);
    }
    public void update(){
        liftController.setPIDF(kP, 0, kD, 0);
        double power = liftController.calculate(lift1.getCurrentPosition());
        liftMotors.set(power+kF);
    }
    public void open(){
        deposit.setPosition(0.55);
    }
    public void close(){
        deposit.setPosition(0.93);
    }
    public void half_open(){
        deposit.setPosition(0.65);
    }
    public void setPower(double power){
        liftMotors.set(power);
    }

    public void resetEncoder(){
        liftMotors.resetEncoder();
    }

    public double getDistance(){
        return liftend.getDistance(DistanceUnit.CM);
    }
    public boolean is_down(){
        return this.getDistance() < down_cutoff;
    }
}
