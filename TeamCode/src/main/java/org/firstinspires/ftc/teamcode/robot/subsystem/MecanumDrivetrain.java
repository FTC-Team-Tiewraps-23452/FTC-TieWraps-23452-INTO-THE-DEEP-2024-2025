package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


/**
 * This file is an example of how to define a mecanum drivetrain in your code.
 * It is structured in the same way as the ExampleSubsystem

 * Different functions of the drivetrain have been defined, but not yet fully worked out
 */
public class MecanumDrivetrain {

    //Declare motor and servo objects
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
        /*
         * These lines of code links the DcMotors to the ports on the control/expansion hub
         * with the corresponding labels (deviceName)
         * This 'labeling' can be done on the Driver Station by clicking on the three dots
         * in the upper right corner and then going to 'Configure Robot'
         */
        rightFront = hardwareMap.get(DcMotor.class, "motor0");
        leftFront =  hardwareMap.get(DcMotor.class, "motor1");
        rightBack =  hardwareMap.get(DcMotor.class, "motor2");
        leftBack =  hardwareMap.get(DcMotor.class, "motor3");

        /*
         * Normally a DC motors runs in the clockwise direction for positive values
         * If positive values need to correspond to counter clockwise rotation,
         * for example for a drivetrain, the motor can be reversed
         * Usually the left side of the drivetrain needs to be reversed,
         * but this should always be checked to be sure
         */
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the motors to use the integrated encoders
         * This gives a bit more precision while controlling the motors
         */
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* put the motors on brake mode
         * this gives more precision while driving
         */
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    /*
     * After the constructor the functions of the subsystem can be defined.
     * The main function of a drivetrain is, of course, driving.
     * This has been defined below for different types of drivetrains
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

    public void mecanumDrivePosition(int position){
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

        leftFront.setPower(0.5);
        leftBack.setPower(0.5);
        rightFront.setPower(0.5);
        rightBack.setPower(0.5);
    }

    public Boolean isBusy() {
        return leftBack.isBusy() && leftFront.isBusy() && rightBack.isBusy() && rightFront.isBusy();
    }
}