package org.firstinspires.ftc.teamcode.laboratory;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class limelight_lab extends OpMode{

    Limelight3A limelight;

    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop(){
        LLStatus status = limelight.getStatus();
        LLResult result = limelight.getLatestResult();
        if(result.isValid()){

        }
    }

}
