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
     * The constructor should have the same name as the class
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

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * a function to move the mecanum drivetrain with a specific speed
     * with an x, y and rx parameter
     *
     * @param x the speed in the x axis from -1 to 1
     * @param y the speed in the y axis from -1 to 1
     * @param rx the speed to turn around the z axis from -1 to 1
     */
    public void mecanumDrive(double x, double y, double rx){
        leftFront.setPower((y + x + rx));
        leftBack.setPower((y - x + rx));
        rightFront.setPower((y - x - rx));
        rightBack.setPower((y + x - rx));
    }

    /**
     * a function to reset all of the encoders of the drivetrain
     */
    public void mecanumDriveResetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * a function to set a desired tick target to the drivetrain
     *
     * @param tickTarget the tick target for the drivetrain
     */
    public void setTargetPosition(int tickTarget) {
        leftFront.setTargetPosition(tickTarget);
        rightFront.setTargetPosition(tickTarget);
        leftBack.setTargetPosition(tickTarget);
        rightBack.setTargetPosition(tickTarget);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * a function to check if the robot is on its desired position
     *
     * @param position the desired position in ticks
     * @return returns true or false depending on if its on the desired position
     */
    public boolean onPosition(int position){
        return (leftFront.getCurrentPosition() <= position + 5 && leftFront.getCurrentPosition() >= position - 5 &&
                rightFront.getCurrentPosition() <= position + 5 && rightBack.getCurrentPosition() >= position - 5 &&
                leftBack.getCurrentPosition() <= position + 5 && leftBack.getCurrentPosition() >= position - 5 &&
                rightBack.getCurrentPosition() <= position + 5 && rightBack.getCurrentPosition() >= position - 5);
    }

    /**
     * a function to stop all of the drivetrain motors
     */
    public void stopAll(){
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    /**
     * function to read the right front motor position
     *
     * @return returns the current position
     */
    public double rightFrontValues(){
        return rightFront.getCurrentPosition();
    }

    /**
     * function to read the left front motor position
     *
     * @return returns the current position
     */
    public double leftFrontValues(){
        return leftFront.getCurrentPosition();
    }

    /**
     * function to read the right back motor position
     *
     * @return returns the current position
     */
    public double rightBackValues(){
        return rightBack.getCurrentPosition();
    }

    /**
     * function to read the left back motor position
     *
     * @return returns the current position
     */
    public double leftBackValues(){
        return leftBack.getCurrentPosition();
    }
}