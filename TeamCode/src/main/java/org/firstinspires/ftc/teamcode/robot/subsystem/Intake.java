package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final CRServo intakeServo;
    private final DcMotor storeMotor;


    /**
     * This is the constructor of the subsystem
     * This is the function that will be run when the subsystem is created,
     * which happens at the beginning of an OpMode.
     * The constructor should have the same name as the class
     *
     * @param hardwareMap This is the input of the constructor, which will be used
     *                    to link the motors and servos in the code to the motors and servos
     *                    on the actual robot
     */
    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        storeMotor = hardwareMap.get(DcMotor.class, "storeMotor");
        storeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        storeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * a function to set the intake position to go upwards (true) or go downwards (false).
     *
     * @param direction the direction true or false
     */
    public void moveIntakePosition(boolean direction) {
        //if true
        if (direction) {
            storeMotor.setPower(0.2);
            storeMotor.setTargetPosition(2);
        }
        //if false
        if (!direction) {
            storeMotor.setPower(0.2);
            storeMotor.setTargetPosition(384);
        }
        storeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * a function to set the speed of the intake store motor with 1 and -1 being max speed and 0 to stop
     *
     * @param speed the speed for the motor
     */
    public void moveIntake(double speed){
        storeMotor.setPower(speed);
    }

    /**
     * a function to set the speed of the intake servo with 1 and -1 being max speed and 0 to stop
     *
     * @param speed the speed for the servo
     */
    public void setIntakeServoSpeed(double speed){
        intakeServo.setPower(speed);
    }

    /**
     * a function to read the current position of the intake motor
     *
     * @return returns the current position
     */
    public int intakeValues(){
        return storeMotor.getCurrentPosition();
    }
}
