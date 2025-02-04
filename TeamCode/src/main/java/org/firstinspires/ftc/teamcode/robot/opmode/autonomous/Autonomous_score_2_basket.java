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
        drive(-20, 0.1);
        intake.moveIntakePosition(false);
        lift.moveLiftPosition(true);
        sleep(2000);
        drive(-20, 0.1);
        lift.moveServo(0.75);
        sleep(1500);
        lift.moveServo(-0.75);
        drive(70, 0.2);
        lift.moveLiftPosition(false);
        sleep(2000);


    }

    /**
     * function to drive to calculate and run the movement to a desired position
     *
     * @param driveDistance the distance desired to be driven in cm from the robot
     * @param speed the speed for the drivetrain to drive with
     */
    private void drive(double driveDistance, double speed) {
        mecanumDrivetrain.mecanumDriveResetEncoders();

        int tick_target = (int) (driveDistance * TICKS_PER_CENTIMETER);
        mecanumDrivetrain.setTargetPosition(tick_target);

        while (!mecanumDrivetrain.onPosition(tick_target) && opModeIsActive()) {
            mecanumDrivetrain.mecanumDrive(0, speed, 0);
        }
        mecanumDrivetrain.stopAll();
    }

}