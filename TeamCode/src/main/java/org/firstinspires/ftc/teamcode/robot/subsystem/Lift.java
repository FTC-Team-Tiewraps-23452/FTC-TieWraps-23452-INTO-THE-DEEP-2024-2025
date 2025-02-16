package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {

    private final DcMotor liftMotor;
    private final Servo liftServo;


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
    public Lift(HardwareMap hardwareMap){
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        liftServo = hardwareMap.get(Servo.class, "storeBoxServo");

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftServo.setDirection(Servo.Direction.FORWARD);
    }

    /**
     * a function to set the lift position to go upwards (true) or go downwards (false).
     *
     * @param direction the direction true or false
     */
    public void moveLiftPosition(boolean direction) {
        //if true
        if (direction) {
            liftMotor.setTargetPosition(-2541);
            liftMotor.setPower(1);
        }
        //if false
        if (!direction) {
            liftMotor.setTargetPosition(-41);
            liftMotor.setPower(0.5);
        }
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * a function to set the position of the lift servo
     * while moving in reverse when the position is smaller than 0
     * and moving forwards if the value is greater than 0
     *
     * @param position the position for the servo to go to
     */
    public void moveServo(double position){
        if (position < 0) {
            liftServo.setDirection(Servo.Direction.REVERSE);
            liftServo.setPosition(position);

        } else {
            liftServo.setDirection(Servo.Direction.FORWARD);
            liftServo.setPosition(position);
        }
    }

    /**
     * a function to set the speed of the lift motor with 1 and -1 being max speed and 0 to stop
     *
     * @param speed the speed for the motor
     */
    public void moveLift(double speed) {
        liftMotor.setPower(speed);
    }

    /**
     * a function to read the current position of the lift motor
     *
     * @return returns the current position
     */
    public double liftServoValues(){
        return liftServo.getPosition();
    }
    public double liftMotorValues(){
        return liftMotor.getCurrentPosition();
    }
}


