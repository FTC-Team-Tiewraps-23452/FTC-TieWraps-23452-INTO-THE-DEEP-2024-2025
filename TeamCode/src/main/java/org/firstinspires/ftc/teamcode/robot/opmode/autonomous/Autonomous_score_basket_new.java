package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous score basket", group="Linear OpMode")
public class Autonomous_score_basket_new extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private Intake intake;
    private Lift lift;
    private MecanumDrivetrain mecanumDrivetrain;

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


        // cycle 1h
        lift.moveServo(-0.75);
        mecanumDrivetrain.mecanumDrive(0, -0.1, 0);
        intake.moveIntakePosition(false);
        sleep(750);
        mecanumDrivetrain.stopAll();
        lift.moveLiftPosition(true);
        sleep(2000);
        mecanumDrivetrain.mecanumDrive(0, -0.1, 0);
        sleep(800);
        mecanumDrivetrain.stopAll();
        sleep(500);
        lift.moveServo(0.75);
        sleep(1500);
        lift.moveServo(-0.75);
        sleep(1000);
        mecanumDrivetrain.mecanumDrive(0, 0.1, 0);
        sleep(1200);
        mecanumDrivetrain.stopAll();
        sleep(300);
        lift.moveLiftPosition(false);
        sleep(3000);
    }
}
