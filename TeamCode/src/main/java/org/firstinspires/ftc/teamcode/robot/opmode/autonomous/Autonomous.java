package org.firstinspires.ftc.teamcode.robot.opmode.autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystem.MecanumDrivetrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Linear OpMode")
public class Autonomous extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    private IMU imu;
    private Intake intake;
    private Lift lift;
    private MecanumDrivetrain mecanumDrivetrain;

    double WHEEL_CIRCUMFERENCE = 30.1593;
    double ENCODER_RESOLUTION = 537.7;
    double TICKS_PER_CENTIMETER = ENCODER_RESOLUTION / WHEEL_CIRCUMFERENCE;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing");

        imu = hardwareMap.get(IMU.class, "imu");

        mecanumDrivetrain = new MecanumDrivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        rotate(90);
//        drive(50);


    }

    private void drive(double driveDistance) {
        double tick_target = driveDistance / TICKS_PER_CENTIMETER;
        mecanumDrivetrain.mecanumDrivePosition((int)tick_target);
        if (!mecanumDrivetrain.isBusy()) {
            mecanumDrivetrain.mecanumDrive(0, 0, 0);
        }
    }

    private void rotate(double rotaionAngle) {
        imu.resetYaw();
        while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) <= rotaionAngle - 4 || imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= rotaionAngle + 4 && opModeIsActive()) {
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 180 && opModeIsActive()) {
                mecanumDrivetrain.mecanumDrive(0, 0, 0.02);
            }
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) >= 180 && opModeIsActive()) {
                mecanumDrivetrain.mecanumDrive(0, 0, -0.02);
            }
        }
    }
}