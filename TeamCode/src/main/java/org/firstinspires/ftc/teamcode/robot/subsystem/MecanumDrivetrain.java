package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDrivetrain {

    private final DcMotor rightFront;
    private final DcMotor leftFront;
    private final DcMotor rightBack;
    private final DcMotor leftBack;


    /**
     * This is the constructor of the subsystem
     * This is the function that will be run when the subsystem is created,
     * which happens at the beginning of an OpMode.
     * The constructor should have the same name as the class (ExampleMecanumDrivetrain in this case).
     *
     * @param hardwareMap This is the input of the constructor, which will be used
     *                    to link the motors and servos in the code to the motors and servos
     *                    on the actual robot
     */
    public MecanumDrivetrain(HardwareMap hardwareMap){
        rightFront = hardwareMap.get(DcMotor.class, "motor0");
        leftFront =  hardwareMap.get(DcMotor.class, "motor1");
        rightBack =  hardwareMap.get(DcMotor.class, "motor2");
        leftBack =  hardwareMap.get(DcMotor.class, "motor3");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void mecanumDrive(double x, double y, double rx){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setPower((y + x + rx));
        leftBack.setPower((y - x + rx));
        rightFront.setPower((y - x - rx));
        rightBack.setPower((y + x - rx));
    }

    public void mecanumDrivePosition(int position, double speed){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setTargetPosition(position);
        leftBack.setTargetPosition(position);
        rightFront.setTargetPosition(position);
        rightBack.setTargetPosition(position);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    public Boolean isBusy() {
        return leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy();
    }

    public void stopAll(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    public double rightFrontValues(){
        return rightFront.getCurrentPosition();
    }
}