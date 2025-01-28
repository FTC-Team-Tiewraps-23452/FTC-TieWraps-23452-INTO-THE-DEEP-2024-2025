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

    /**
     * a function to move the mecanum drivetrain with a specific speed
     * with an x, y and rx parameter
     *
     * @param x the speed in the x axis from -1 to 1
     * @param y the speed in the y axis from -1 to 1
     * @param rx the speed to turn around the z axis from -1 to 1
     */
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

    /**
     * a function to move the mecanum drivetrain a specific distace with a speed and position
     *
     * @param position the position in cm from the robot
     * @param speed the specific speed to go to the position from -1 to 1
     */
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
     * a function to check if the drivetrain motors are busy
     *
     * @return returns true or false depending on if the drivetrain is busy or not
     */
    public Boolean isBusy() {
        return leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy();
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