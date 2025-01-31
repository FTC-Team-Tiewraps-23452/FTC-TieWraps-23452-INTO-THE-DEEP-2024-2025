package org.firstinspires.ftc.teamcode.robot.opmode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystem.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;


@TeleOp(name="TeleOP-IntoTheDeep-", group="Iterative Opmode")
//@Disabled
public class TeleOP extends OpMode
{
    // Declare timer to keep track of how long the program has been running
    private final ElapsedTime runtime = new ElapsedTime();

    /*
     * Declare subsystems
     * This means that we will say that certain subsystems exist and give them a name,
     * but not yet create them, this will happen in the init() function.
     */
    private MecanumDrivetrain mecanumDrivetrain;
    private Lift lift;
    private Intake intake;

    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Telemetry.addData is used to display variables and text on the Driver Station
        telemetry.addData("Status", "Initializing");

        /*
         * Create all the subsystems
         * Go to the folder 'subsystems' to view the subsystems, which contain more information
         */
        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);
        lift = new Lift(hardwareMap);
        intake = new Intake(hardwareMap);

        // Tell the driver that initialization is complete via the Driver Station
        telemetry.addData("Status", "Initialized");
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        // Restart the timer
        runtime.reset();
    }

    /**
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("drivetrain", "right Front: " + mecanumDrivetrain.rightFrontValues());
        telemetry.addData("drivetrain", "right Back: " + mecanumDrivetrain.rightBackValues());
        telemetry.addData("drivetrain", "left Front: " + mecanumDrivetrain.leftFrontValues());
        telemetry.addData("drivetrain", "left back: " + mecanumDrivetrain.leftBackValues());

        telemetry.addData("intake", "position" + intake.intakeValues());
        telemetry.addData("lift", "position" + lift.liftValues());


        //drivetrain
        if (gamepad1.left_bumper){
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            mecanumDrivetrain.mecanumDrive(x, y, rx);
        }if (gamepad1.left_trigger !=0){
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            mecanumDrivetrain.mecanumDrive(x / 15, y / 15, rx / 15);
        }else {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            mecanumDrivetrain.mecanumDrive(x / 5,y / 5,rx / 5);
        }


        //lift
        if (gamepad2.left_stick_y < 0){
            lift.moveLift(gamepad2.left_stick_y);
        } else {
            lift.moveLift(gamepad2.left_stick_y * 0.5);
        }

        //bakje lift
        if (gamepad1.right_bumper) {
            lift.moveServo(0.80);
        } else {
            lift.moveServo(1);
        }

        //Intake servo
        if (gamepad2.right_bumper) {
            intake.setIntakeServoSpeed(1.0);
        } else if (gamepad2.left_bumper){
            intake.setIntakeServoSpeed(-1.0);
        } else {
            intake.setIntakeServoSpeed(0.0);
        }

        //intake
        intake.moveIntake(gamepad2.right_stick_y / 5);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /**
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}