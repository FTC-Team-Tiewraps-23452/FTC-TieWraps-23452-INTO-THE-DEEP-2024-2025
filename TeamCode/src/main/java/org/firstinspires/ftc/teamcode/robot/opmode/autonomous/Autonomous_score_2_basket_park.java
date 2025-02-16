package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous score 2 basket park", group="Linear OpMode")
public class Autonomous_score_2_basket_park extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private Intake intake;
    private Lift lift;
    private MecanumDrivetrain mecanumDrivetrain;

    double WHEEL_CIRCUMFERENCE = 30.1593;
    double ENCODER_RESOLUTION = 537.7;
    double TICKS_PER_CENTIMETER = ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE;
    double kP = 1;
    int errorMargin = 50;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // cycle 1
        drive(10, kP, 0.1);
        intake.moveIntakePosition(false);
        sleep(500);
        lift.moveLiftPosition(true);
        sleep(2000);
        drive(20, kP, 0.1);
        sleep(1500);
        lift.moveServo(0.75);
        sleep(1500);
        lift.moveServo(-0.75);
        drive(-20, kP, 0.1);
        sleep(200);
        lift.moveLiftPosition(false);
        sleep(5000);
        intake.moveIntakePosition(true);
        drive(-165, kP/4, 0.05);

//        // cycle 2
//        mecanumDrivetrain.mecanumDrive(0, 0, 0.2);
//        sleep(250);
//        mecanumDrivetrain.stopAll();
//        sleep(500);
//        intake.setIntakeServoSpeed(1);
//        drive(10, kP, 0.1);
//        sleep(500);
//        intake.moveIntakePosition(true);
//        sleep(500);
//        intake.setIntakeServoSpeed(-1);
//        sleep(200);
//        intake.setIntakeServoSpeed(0);
//        drive(-90, kP, 0.1);
//        mecanumDrivetrain.mecanumDrive(0, 0, -0.2);
//        sleep(250);
//        mecanumDrivetrain.mecanumDrive(0.2, 0, 0);
//        sleep(400);
//        mecanumDrivetrain.stopAll();
//        sleep(500);
//        drive(-60, kP, 0.1);
//        lift.moveLiftPosition(true);
//        sleep(500);
//        drive(-10, kP, 0.1);
//        lift.moveServo(0.75);
//        sleep(1500);
//        lift.moveServo(-0.75);
//        drive(-20, kP, 0.1);
//        sleep(200);
//        lift.moveLiftPosition(false);
//        sleep(5000);


    }

    /**
     * Function to calculate and execute the movement of the robot to a desired position.
     *
     * @param driveDistance the distance that the robot should drive from its current position
     * @param kP the proportional gain used in the PID controller to adjust the speed based on the remaining distance
     * @param maxSpeed the maximum speed that the drivetrain can achieve while driving
     */
    private void drive(double driveDistance, double kP, double maxSpeed) {
        mecanumDrivetrain.mecanumDriveResetEncoders();

        int tick_target = (int) (driveDistance * TICKS_PER_CENTIMETER);
        mecanumDrivetrain.setTargetPosition(-tick_target);

        boolean leftBackDone = false, leftFrontDone = false, rightBackDone = false, rightFrontDone = false;

        while (opModeIsActive()){
//                (!(leftBackDone && leftFrontDone && rightBackDone && rightFrontDone)) && opModeIsActive()) {
//            if (!leftBackDone && Math.abs(mecanumDrivetrain.leftBackValues() - tick_target) <= errorMargin) {
//                mecanumDrivetrain.stopLeftBack();
//                leftBackDone = true;
//            }
//            if (!leftFrontDone && Math.abs(mecanumDrivetrain.leftFrontValues() - tick_target) <= errorMargin) {
//                mecanumDrivetrain.stopLeftFront();
//                leftFrontDone = true;
//            }
//            if (!rightBackDone && Math.abs(mecanumDrivetrain.rightBackValues() - tick_target) <= errorMargin) {
//                mecanumDrivetrain.stopRightBack();
//                rightBackDone = true;
//            }
//            if (!rightFrontDone && Math.abs(mecanumDrivetrain.rightFrontValues() - tick_target) <= errorMargin) {
//                mecanumDrivetrain.stopRightFront();
//                rightFrontDone = true;
//            }

            double currentDistance = (mecanumDrivetrain.leftBackValues() + mecanumDrivetrain.leftFrontValues() +
                    mecanumDrivetrain.rightBackValues() + mecanumDrivetrain.rightFrontValues()) / 4.0;

            double remainingDistance = tick_target - currentDistance;
            double speed = kP * remainingDistance;

            speed = Math.max(-maxSpeed, Math.min(speed, maxSpeed));

            if (Math.abs(remainingDistance) <= errorMargin) {
                speed = 0;
                mecanumDrivetrain.stopAll();
                return;
            }

            mecanumDrivetrain.mecanumDrive(0, speed, 0);

            telemetry.addData("position", "leftBack: " + mecanumDrivetrain.leftBackValues());
            telemetry.addData("position", "leftFront: " + mecanumDrivetrain.leftFrontValues());
            telemetry.addData("position", "rightFront: " + mecanumDrivetrain.rightFrontValues());
            telemetry.addData("position", "rightBack: " + mecanumDrivetrain.rightBackValues());
            telemetry.addData("target position", tick_target);
            telemetry.addData("remaining distance", remainingDistance);
            telemetry.update();
        }

        mecanumDrivetrain.stopAll();
    }



}