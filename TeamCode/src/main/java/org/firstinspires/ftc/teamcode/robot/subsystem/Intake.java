package org.firstinspires.ftc.teamcode.robot.subsystem;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    private final CRServo intakeServo;
    private final DcMotor storeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        storeMotor = hardwareMap.get(DcMotor.class, "storeMotor");
        storeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        storeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public class armOut implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                storeMotor.setPower(0.2);
                initialized = true;
            }

            double pos = storeMotor.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos > -3) {
                return true;
            } else {
                storeMotor.setPower(0);
                return false;
            }
        }
    }
    public Action armOut() {
        return new armOut();
    }

    public class armIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                storeMotor.setPower(0.2);
                initialized = true;
            }

            double pos = storeMotor.getCurrentPosition();
            packet.put("liftPos", pos);
            if (pos < 416) {
                return true;
            } else {
                storeMotor.setPower(0);
                return false;
            }
        }
    }
    public Action armIn() {
        return new armIn();
    }

    public class intake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServo.setPower(1);
            return false;
        }
    }
    public Action intake() {
        return new intake();
    }

    public class outtake implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeServo.setPower(-1);
            return false;
        }
    }
    public Action outtake() {
        return new outtake();
    }

    public void moveIntake(double speed){
        storeMotor.setPower(speed);
    }

    public void setIntakeServoSpeed(double speed){
        intakeServo.setPower(speed);
    }

    public int intakeValues(){
        return storeMotor.getCurrentPosition();
    }
}
