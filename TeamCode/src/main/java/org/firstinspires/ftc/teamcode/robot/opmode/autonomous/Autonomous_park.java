package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous park", group="Linear OpMode")
public class Autonomous_park extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();
    private MecanumDrivetrain mecanumDrivetrain;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");


        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        mecanumDrivetrain.mecanumDrive(0, 0.4, 0);
        sleep(600);
        mecanumDrivetrain.mecanumDrive(0, 0, 0);

        }
}