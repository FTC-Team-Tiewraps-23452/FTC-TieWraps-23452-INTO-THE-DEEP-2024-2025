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


     // cycle 1
     mecanumDrivetrain.mecanumDrive(-1, 0, 0);
     sleep(950);
     mecanumDrivetrain.mecanumDrive(0.75, 0, 0);
     sleep(200);
     mecanumDrivetrain.stopAll();
     mecanumDrivetrain.mecanumDrive(0, 0, -0.5);
     sleep(400);
     mecanumDrivetrain.stopAll();
     moveIntake(true);
     sleep(500);
     moveLift(true);
     mecanumDrivetrain.mecanumDrive(0, -0.2, 0);
     sleep(300);
     mecanumDrivetrain.stopAll();
     lift.setServoPosition(0);
     sleep(2500);
     lift.setServoPosition(0.2);
     sleep(1000);
     mecanumDrivetrain.mecanumDrive(0, 0.2, 0);
     sleep(600);
     mecanumDrivetrain.stopAll();
     moveLift(false);

     //cycle 2
//         mecanumDrivetrain.mecanumDrive(0, 0.3, 0);
//         sleep(500);
//         mecanumDrivetrain.stop();
//         sleep();
//         mecanumDrivetrain.mecanumDrive();
//         sleep();
//         mecanumDrivetrain.stop();
//         sleep();
//         intake.setIntakeServoSpeed(1);
//         mecanumDrivetrain.mecanumDrive(0, 0.2, 0);
//         sleep();
//         moveIntake(true);
//         intake.setIntakeServoSpeed(-1);
//         sleep();
//         moveIntake(false);
//         mecanumDrivetrain.mecanumDrive(0, 0, 0.2);
//         sleep();
//         mecanumDrivetrain.stop();
 }

 // gives the direction for the lift true is up and false is down
 private void moveLift(boolean direction) {
     if (direction){
         lift.setLiftSpeed(-1);
         sleep(1900);
         lift.setLiftSpeed(0);
     } else {
         lift.setLiftSpeed(0.5);
         sleep(2600);
         lift.setLiftSpeed(0);
     }
 }

 //gives the direction for the intake true is to intake false is to store
 private void moveIntake(boolean direction) {
     if (direction){
         intake.setIntakeSpeed(0.2);
         sleep(1200);
         intake.setIntakeSpeed(0);
     } else{
         intake.setIntakeSpeed(-0.2);
         sleep(1200);
         intake.setIntakeSpeed(0);
     }
 }
}
