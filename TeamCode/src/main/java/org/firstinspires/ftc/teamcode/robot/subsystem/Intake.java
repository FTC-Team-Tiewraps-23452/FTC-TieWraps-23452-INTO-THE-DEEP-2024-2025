package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final CRServo intakeServo;
    private final DcMotor storeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        storeMotor = hardwareMap.get(DcMotor.class, "storeMotor");
        storeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        storeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //TODO tune the values
    //true is to intake position false is to store position
    public void moveIntakePosition(boolean direction) {
        //if true
        if (direction) {
            storeMotor.setPower(0.2);
            storeMotor.setTargetPosition(-423);
        }
        //if false
        else {
            storeMotor.setPower(0.2);
            storeMotor.setTargetPosition(-14);
        }
        storeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveIntake(double speed){
        storeMotor.setPower(speed);
    }

    public void setIntakeServoSpeed(double speed){
        intakeServo.setPower(speed);
    }

    public int intakeValues(){
        return storeMotor.getCurrentPosition();
    }
}
