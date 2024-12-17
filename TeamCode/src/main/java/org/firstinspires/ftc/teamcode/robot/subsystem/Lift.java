package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
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

        liftServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setLiftSpeed(double speed){
        liftMotor.setPower(speed);
    }

    public void setServoPosition(double position){
        liftServo.setPosition(position);
    }

    public void setMotorPosition(int position) {
        liftMotor.setTargetPosition(position);
        liftMotor.setPower(0.2);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int liftValues(){
        return liftMotor.getCurrentPosition();
    }
}


