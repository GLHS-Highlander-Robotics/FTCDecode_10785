package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class launcher {
    DcMotorEx launcher;
    double kF = 0.19, kP = 0.07;

    public launcher(HardwareMap hardwareMap, Telemetry telemetry){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("LAUNCHER", "INITIALIZED");
        telemetry.update();
    }

    public void powerFlywheel(double targetSpeed){
        if(targetSpeed == 0){
            launcher.setPower(kP*(targetSpeed-launcher.getVelocity()));
        }
        else launcher.setPower(kF+kP*(targetSpeed-launcher.getVelocity()));
    }
}
