package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.constants;

public class intake {
    private DcMotorEx intakeMotor;

    public intake(HardwareMap hardwareMap, Telemetry telemetry){
        intakeMotor = hardwareMap.get(DcMotorEx.class,constants.INTAKE_MOTOR_NAME);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("INTAKE: ", "INITIALIZED");
    }

    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }
}
