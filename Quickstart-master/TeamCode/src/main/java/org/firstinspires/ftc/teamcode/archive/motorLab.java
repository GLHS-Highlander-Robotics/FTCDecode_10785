package org.firstinspires.ftc.teamcode.archive;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Configurable
@TeleOp
public class motorLab extends OpMode {
    DcMotorEx motorEx;
    VoltageSensor sensor;
    ElapsedTime time;
    public static String deviceName = "FL";
    public static double voltage = 1;
    public TelemetryManager telemetryM;


    @Override
    public void init(){
        motorEx = hardwareMap.get(DcMotorEx.class, deviceName);
        motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        time = new ElapsedTime();
        telemetryM.addData("Encoder Reading", motorEx.getVelocity(AngleUnit.RADIANS));
        telemetryM.update();
    }

    public void loop(){
        motorEx.setPower(voltage/sensor.getVoltage());
        telemetryM.addData("Encoder Reading", motorEx.getVelocity(AngleUnit.RADIANS));
        telemetryM.update();
    }

}
