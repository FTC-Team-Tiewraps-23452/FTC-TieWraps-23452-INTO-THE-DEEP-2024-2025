package org.firstinspires.ftc.teamcode.robot.subsystem;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Climber {

    private final DcMotor climberMotor;

    public Climber(HardwareMap hardwareMap){
        climberMotor = hardwareMap.get(DcMotor.class, "climberMotor");
        climberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climberMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setClimberSpeed(double speed){climberMotor.setPower(speed);
    }
}