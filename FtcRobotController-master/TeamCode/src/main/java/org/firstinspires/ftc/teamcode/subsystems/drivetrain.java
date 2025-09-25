package org.firstinspires.ftc.teamcode.subsystems;
import org.firstinspires.ftc.teamcode.constants.constants;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.math.Vector2D;


public class drivetrain {
    DcMotor FL, FR, BL, BR;


    public drivetrain(HardwareMap hardwareMap, Telemetry telemetry){
        FL = hardwareMap.get(DcMotor.class, constants.MOTORNAMES[0]);
        FR = hardwareMap.get(DcMotor.class, constants.MOTORNAMES[1]);
        BL = hardwareMap.get(DcMotor.class, constants.MOTORNAMES[2]);
        BR = hardwareMap.get(DcMotor.class, constants.MOTORNAMES[3]);

        telemetry.addData("DRIVETRAIN", "INITIALIZED");
    }

    public void setRunModes(DcMotor.RunMode runMode){
        FL.setMode(runMode);
        FR.setMode(runMode);
        BL.setMode(runMode);
        BR.setMode(runMode);
    }

    public void driveBot(double fwd, double strf, double turn){
        double max = Math.abs(fwd)+Math.abs(strf)+Math.abs(turn);
        fwd = fwd/max;
        strf = strf/max;
        turn = turn/max;

        FL.setPower(fwd+strf-turn);
        FR.setPower(fwd-strf+turn);
        BL.setPower(fwd-strf-turn);
        BR.setPower(fwd+strf+turn);
    }

    public void driveBot(Vector2D translational, double turn){
        double fwd = translational.y;
        double strf = translational.x;
        double max = Math.abs(fwd)+Math.abs(strf)+Math.abs(turn);
        fwd = fwd/max;
        strf = strf/max;
        turn = turn/max;

        FL.setPower(fwd+strf-turn);
        FR.setPower(fwd-strf+turn);
        BL.setPower(fwd-strf-turn);
        BR.setPower(fwd+strf+turn);
    }
}
