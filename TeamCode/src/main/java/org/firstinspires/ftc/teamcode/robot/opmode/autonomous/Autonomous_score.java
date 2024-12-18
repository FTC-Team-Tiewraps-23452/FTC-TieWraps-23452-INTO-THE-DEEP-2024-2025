 package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.IMU;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
 import org.firstinspires.ftc.teamcode.robot.subsystem.Lift;
 import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;

 @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous score", group="Linear OpMode")
 public class Autonomous_score extends LinearOpMode {

     private final ElapsedTime runtime = new ElapsedTime();

     private IMU imu;
     private Intake intake;
     private Lift lift;
     private MecanumDrivetrain mecanumDrivetrain;

     @Override
     public void runOpMode() {
         telemetry.addData("Status", "Initializing");

         //        imu = hardwareMap.get(IMU.class, "imu");

         mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);
         intake = new Intake(hardwareMap);
         lift = new Lift(hardwareMap);

         telemetry.addData("Status", "Initialized");
         telemetry.update();

         waitForStart();
         runtime.reset();

         mecanumDrivetrain.mecanumDrive(-1, 0, 0);
         sleep(950);
         mecanumDrivetrain.mecanumDrive(0.75, 0, 0);
         sleep(200);
         mecanumDrivetrain.mecanumDrive(0, 0, 0);
     }
 }