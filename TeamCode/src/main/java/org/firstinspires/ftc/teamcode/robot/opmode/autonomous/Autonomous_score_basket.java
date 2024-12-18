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
         mecanumDrivetrain.mecanumDrive(0, 0, 0);
         mecanumDrivetrain.mecanumDrive(0, 0, -0.5);
         sleep(400);
         mecanumDrivetrain.mecanumDrive(0, 0, 0);
         intake.setIntakeSpeed(0.2);
         sleep(1200);
         intake.setIntakeSpeed(0);
         lift.setLiftSpeed(-1);
         sleep(1900);
         lift.setLiftSpeed(0);
         mecanumDrivetrain.mecanumDrive(0, -0.2, 0);
         sleep(300);
         mecanumDrivetrain.mecanumDrive(0, 0, 0);
         lift.setServoPosition(0);
         sleep(5000);
         lift.setServoPosition(0.2);
         sleep(1000);
         mecanumDrivetrain.mecanumDrive(0, 0.2, 0);
         sleep(600);
         mecanumDrivetrain.mecanumDrive(0, 0, 0);
         lift.setLiftSpeed(0.5);
         sleep(2900);
         lift.setLiftSpeed(0);
     }
 }