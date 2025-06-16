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
     * a function to reset the internal imu
     */
    public void resetIMU(){
        imu.resetYaw();
    }
}