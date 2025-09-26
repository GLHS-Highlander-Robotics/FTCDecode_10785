package org.firstinspires.ftc.teamcode.laboratory;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TURRET PROTOTYPE", group = "TESTS")
public class turret_prototype extends OpMode {

    DcMotorEx turretMotor;

    @Override
    public void init(){
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret motor");
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop(){
        if(gamepad1.b){
            turretMotor.setVelocity(500*2*Math.PI, AngleUnit.RADIANS);
        }
        else if(gamepad1.a){
            turretMotor.setVelocity(1000*2*Math.PI, AngleUnit.RADIANS);
        }
        else if(gamepad1.x){
            turretMotor.setVelocity(2000*2*Math.PI, AngleUnit.RADIANS);
        }
        else{
            turretMotor.setVelocity(0, AngleUnit.RADIANS);
        }
    }
}
