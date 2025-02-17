package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous score basket", group="Linear OpMode")
public class Autonomous_score_basket extends LinearOpMode {

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
        mecanumDrivetrain.mecanumDrive(-1, 0, 0);
        sleep(950);
        mecanumDrivetrain.mecanumDrive(0.75, 0, 0);
        sleep(200);
        mecanumDrivetrain.stopAll();
        mecanumDrivetrain.mecanumDrive(0, 0, -0.5);
        sleep(400);
        mecanumDrivetrain.stopAll();
        intake.moveIntakePosition(false);
        sleep(500);
        lift.moveLiftPosition(true);
        sleep(2000);
        mecanumDrivetrain.mecanumDrive(0, -0.2, 0);
        sleep(400);
        mecanumDrivetrain.stopAll();
        sleep(3000);
        lift.moveServo(0.75);
        sleep(1500);
        lift.moveServo(-0.75);
        sleep(1000);
        mecanumDrivetrain.mecanumDrive(0, 0.2, 0);
        sleep(600);
        mecanumDrivetrain.stopAll();
        sleep(100);
        lift.moveLiftPosition(false);
        sleep(10000);
    }
}
