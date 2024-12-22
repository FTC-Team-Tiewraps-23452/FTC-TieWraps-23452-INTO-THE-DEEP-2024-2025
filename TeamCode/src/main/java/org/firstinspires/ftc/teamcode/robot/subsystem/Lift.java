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

    //TODO tune the values
    //true is up and false is down
    public void moveLift(boolean direction) {
        //if true
        if (direction) {
            liftMotor.setTargetPosition(1500);
            liftMotor.setPower(1);
        }
        //if false
        else {
            liftMotor.setTargetPosition(0);
            liftMotor.setPower(-0.5);
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setLiftSpeed(double speed){
        liftMotor.setPower(speed);
    }

    public void moveServo(boolean direction){
        if (!direction) {
            liftServo.setPosition(0.2);
        }
        else {
            liftServo.setPosition(0);
        }
    }

    public int liftValues(){
        return liftMotor.getCurrentPosition();
    }
}


