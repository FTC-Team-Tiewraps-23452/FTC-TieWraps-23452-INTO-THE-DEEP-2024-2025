package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class MecanumDrivetrain {

    private final DcMotor rightFront;
    private final DcMotor leftFront;
    private final DcMotor rightBack;
    private final DcMotor leftBack;
    private final IMU imu;



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

        imu = hardwareMap.get(IMU.class, "imu");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

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
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) * 1.1;
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        leftFront.setPower((rotY + rotX + rx) / denominator);
        leftBack.setPower((rotY - rotX + rx) / denominator);
        rightFront.setPower((rotY - rotX - rx) / denominator);
        rightBack.setPower((rotY + rotX - rx) / denominator);
    }

    /**
     * a function to reset all of the encoders of the drivetrain
     */
    public void mecanumDriveResetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        return (leftFront.getCurrentPosition() <= position + 25 && leftFront.getCurrentPosition() >= position - 25 &&
                rightFront.getCurrentPosition() <= position + 25 && rightBack.getCurrentPosition() >= position - 25 &&
                leftBack.getCurrentPosition() <= position + 25 && leftBack.getCurrentPosition() >= position - 25 &&
                rightBack.getCurrentPosition() <= position + 25 && rightBack.getCurrentPosition() >= position - 25);
    }

    /**
     * a function to reset the internal imu
     */
    public void resetIMU(){
        imu.resetYaw();
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

    /**
     * function to stop the left back motor
     */
    public void stopLeftBack(){
        leftBack.setPower(0);
    }

    /**
     * a function to stop the left front motor
     */
    public void stopLeftFront(){
        leftFront.setPower(0);
    }

    /**
     * a function to stop the right back motor
     */
    public void stopRightBack(){
        rightBack.setPower(0);
    }

    /**
     * a function to stop the right front motor
     */
    public void stopRightFront(){
        rightFront.setPower(0);
    }
}