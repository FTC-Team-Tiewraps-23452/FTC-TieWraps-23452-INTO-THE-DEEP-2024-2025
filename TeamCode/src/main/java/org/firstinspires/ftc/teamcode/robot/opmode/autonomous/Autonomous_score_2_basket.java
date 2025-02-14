package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystem.*;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous score 2 basket", group="Linear OpMode")
public class Autonomous_score_2_basket extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private Intake intake;
    private Lift lift;
    private MecanumDrivetrain mecanumDrivetrain;

    double WHEEL_CIRCUMFERENCE = 30.1593;
    double ENCODER_RESOLUTION = 537.7;
    double TICKS_PER_CENTIMETER = ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE;
    double kP = 0.5;
    int errorMargin = 5;


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

        lift.moveServo(-0.75);
        drive(20, kP, 0.2);
        intake.moveIntakePosition(false);
        lift.moveLiftPosition(true);
        sleep(2000);
        drive(20, kP, 0.2);
        lift.moveServo(0.75);
        sleep(1500);
        lift.moveServo(-0.75);
        drive(-70, kP, 0.2);
        lift.moveLiftPosition(false);
        sleep(2000);

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
        mecanumDrivetrain.setTargetPosition(tick_target);

        boolean leftBackDone = false, leftFrontDone = false, rightBackDone = false, rightFrontDone = false;

        while ((!(leftBackDone && leftFrontDone && rightBackDone && rightFrontDone)) && opModeIsActive()) {
            if (!leftBackDone && (mecanumDrivetrain.leftBackValues() >= tick_target - errorMargin &&
                    mecanumDrivetrain.leftBackValues() <= tick_target + errorMargin)) {
                mecanumDrivetrain.stopLeftBack();
                leftBackDone = true;
            }
            if (!leftFrontDone && (mecanumDrivetrain.leftFrontValues() >= tick_target - errorMargin &&
                    mecanumDrivetrain.leftFrontValues() <= tick_target + errorMargin)) {
                mecanumDrivetrain.stopLeftFront();
                leftFrontDone = true;
            }
            if (!rightBackDone && (mecanumDrivetrain.rightBackValues() >= tick_target - errorMargin &&
                    mecanumDrivetrain.rightBackValues() <= tick_target + errorMargin)) {
                mecanumDrivetrain.stopRightBack();
                rightBackDone = true;
            }
            if (!rightFrontDone && (mecanumDrivetrain.rightFrontValues() >= tick_target - errorMargin &&
                    mecanumDrivetrain.rightFrontValues() <= tick_target + errorMargin)) {
                mecanumDrivetrain.stopRightFront();
                rightFrontDone = true;
            }

            if (opModeIsActive()) {
                double currentDistance = (mecanumDrivetrain.leftBackValues() + mecanumDrivetrain.leftFrontValues() +
                        mecanumDrivetrain.rightBackValues() + mecanumDrivetrain.rightFrontValues()) / 4.0;

                double remainingDistance = tick_target - currentDistance;
                double speed = kP * (remainingDistance / tick_target);

                speed = Math.max(-maxSpeed, Math.min(speed, maxSpeed));

                mecanumDrivetrain.mecanumDrive(0, speed, 0);
            }

            telemetry.addData("position", "leftBack: " + mecanumDrivetrain.leftBackValues());
            telemetry.addData("position", "leftFront: " + mecanumDrivetrain.leftFrontValues());
            telemetry.addData("position", "rightFront: " + mecanumDrivetrain.rightFrontValues());
            telemetry.addData("position", "rightBack: " + mecanumDrivetrain.rightBackValues());
            telemetry.addData("target position", tick_target);
            telemetry.update();
        }
    }
}