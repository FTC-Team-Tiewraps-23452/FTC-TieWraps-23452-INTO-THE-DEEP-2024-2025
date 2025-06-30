package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.opencv.core.Mat;

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
     * General method to move intake to a given target position with proportional slowdown.
     *
     * @param targetPos the given target position.
     */
    public void moveIntakeToPosition(int targetPos) {
        double currentPos = storeMotor.getCurrentPosition();
        double error = targetPos - currentPos;

        double kP = 0.06;
        double power = kP * error;

        power = Math.max(-0.3, Math.min(0.3, power));

        storeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        storeMotor.setPower(power);

        if (Math.abs(error) < 10) {
            storeMotor.setPower(0);
        }
    }

    /**
     * Moves intake upwards to predefined position.
     */
    public void moveIntakeUp() {
        moveIntakeToPosition(-380);
    }

    /**
     * Moves intake downwards to predefined position.
     */
    public void moveIntakeDown() {
        moveIntakeToPosition(-70);
    }


    /**
     * a function to set the intake servo in.
     */
    public void intakeServoIn(){
        intakeServo.setPower(1);
    }

    /**
     * a function to set the intake servo out.
     */
    public void intakeServoOut(){
        intakeServo.setPower(-1);
    }

    /**
     * a function to set the intake servo off.
     */
    public void intakeServoOff(){
        intakeServo.setPower(0);
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
