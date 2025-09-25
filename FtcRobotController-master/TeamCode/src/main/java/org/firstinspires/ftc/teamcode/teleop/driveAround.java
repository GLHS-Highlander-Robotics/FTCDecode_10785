package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain;

@TeleOp(name="drive around!!!")
public class driveAround extends OpMode {
    drivetrain drivetrain;

    @Override
    public void init(){
        drivetrain = new drivetrain(hardwareMap,telemetry);
        drivetrain.setRunModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        drivetrain.driveBot(Math.pow(gamepad1.left_stick_y, 3),Math.pow(gamepad1.left_stick_x, 3),Math.pow(-gamepad1.right_stick_x, 3));
    }
}
