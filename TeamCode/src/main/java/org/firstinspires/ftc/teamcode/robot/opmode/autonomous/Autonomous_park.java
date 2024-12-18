 package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.IMU;
 import com.qualcomm.robotcore.util.ElapsedTime;

 import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
 import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
 import org.firstinspires.ftc.teamcode.robot.subsystem.Lift;
 import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;

 @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous park", group="Linear OpMode")
 public class Autonomous_park extends LinearOpMode {

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

//        double xRotation = 0;
//        double yRotation = 0;
//        double zRotation = 0;
//        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
//        imu.initialize(new IMU.Parameters(orientationOnRobot));

         telemetry.addData("Status", "Initialized");
         telemetry.update();

         waitForStart();
         runtime.reset();

         mecanumDrivetrain.mecanumDrive(0, 1, 0);
         sleep(800 );
         mecanumDrivetrain.mecanumDrive(0, 0, 0);

     }

     private void drive(double driveDistance) {
     }

     private void rotate(double rotaionAngle) {
         if (imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) != rotaionAngle) {
             if (imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) < 90) {
                 mecanumDrivetrain.mecanumDrive(0, 0, 0.5);
             } if (imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES) >= 90) {
                 mecanumDrivetrain.mecanumDrive(0, 0, -0.5);
             }
         } else {
             mecanumDrivetrain.mecanumDrive(0, 0, 0);
         }
     }
 }