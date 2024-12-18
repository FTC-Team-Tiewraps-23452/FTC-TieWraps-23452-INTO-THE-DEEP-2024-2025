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
     * The constructor should have the same name as the class (ExampleSubsystem in this case).
     *
     * @param hardwareMap This is the input of the constructor, which will be used
     *                    to link the motors and servos in the code to the motors and servos
     *                    on the actual robot
     */
    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        storeMotor = hardwareMap.get(DcMotor.class, "storeMotor");

        storeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // set the motor's zero power behavior to brake
        storeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    public void setIntakeServoSpeed(double speed){
        intakeServo.setPower(speed);
    }

    public void setIntakeSpeed(double speed){
        storeMotor.setPower(speed);
    }

    public int intakeValues(){
        return storeMotor.getCurrentPosition();
    }
}
