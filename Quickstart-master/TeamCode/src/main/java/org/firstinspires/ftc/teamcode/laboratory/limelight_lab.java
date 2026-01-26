package org.firstinspires.ftc.teamcode.laboratory;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;

@Config
@Configurable
@TeleOp(name = "limelight lab", group = "lab")
public class limelight_lab extends OpMode{

    Limelight3A limelight;
    drivetrain drivetrain;
    static double kP = 0.015, kD = 0.01;

    @Override
    public void init(){
        drivetrain = new drivetrain(hardwareMap, telemetry, false);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addData("Limelight", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop(){
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        if(result.isValid()){
            telemetry.addData("ty", result.getTy());
            telemetry.update();
            if(gamepad1.a){
                drivetrain.targetAngle(0,0, 0 ,kP, kD, result.getTy(), 0.10);
            }
            else drivetrain.stopBot();
        }
    }

}
