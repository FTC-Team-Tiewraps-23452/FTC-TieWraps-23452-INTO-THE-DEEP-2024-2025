package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    private final DcMotor liftMotor;
    private final Servo liftServo;



    public Lift(HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftServo = hardwareMap.get(Servo.class, "storeBoxServo");

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        liftServo.setDirection(Servo.Direction.FORWARD);
    }

    //true is up and false is down
    public void moveLiftPosition(boolean direction) {
        //if false
        if (direction) {
            liftMotor.setTargetPosition(-5800);
            liftMotor.setPower(1);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //if true
        else if (!direction){
            liftMotor.setTargetPosition(-3200);
            liftMotor.setPower(-0.5);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void moveServo(double position){
        if (position < 0) {
            liftServo.setDirection(Servo.Direction.REVERSE);
            liftServo.setPosition(position);

        } else {
            liftServo.setDirection(Servo.Direction.FORWARD);
            liftServo.setPosition(position);
        }
    }

    public void moveLift(double speed) {
        liftMotor.setPower(speed);
    }

    public double liftValues(){
        return liftServo.getPosition();
    }
}


