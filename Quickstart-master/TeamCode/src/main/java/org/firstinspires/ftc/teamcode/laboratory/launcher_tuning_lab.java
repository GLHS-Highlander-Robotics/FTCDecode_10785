package org.firstinspires.ftc.teamcode.laboratory;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Configurable
@TeleOp(name = "launcher lab :D")
public class launcher_tuning_lab extends OpMode {

    DcMotorEx launcher;

    double presets[] = {10, 1, 0.1, 0.01, 0.001, 0.0001};
    int presetIndex = 0;
    static double targetSpeed = 600;
    boolean a, b, x, y, up, dn, lt, rt;
    static double kF = 0.19, kP = 0.07;
    boolean setting_kP = false;

    @Override
    public void init(){
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        if (gamepad1.dpad_right && !rt){
            presetIndex++;
        }
        if (!lt && gamepad1.dpad_left &&presetIndex>1){
            presetIndex--;
        }

        if(gamepad1.x && !x){
            setting_kP = !setting_kP;
        }

        if(gamepad1.dpad_up && !up){
            if(setting_kP){
                kP += presets[presetIndex%presets.length];
            }
            else kF += presets[presetIndex%presets.length];
        }

        if(gamepad1.dpad_down && !dn){
            if(setting_kP){
                kP -= presets[presetIndex%presets.length];
            }
            else kF -= presets[presetIndex%presets.length];
        }

        a = gamepad1.a;
        b = gamepad1.b;
        x = gamepad1.x;
        y = gamepad1.y;
        up = gamepad1.dpad_up;
        dn = gamepad1.dpad_down;
        lt = gamepad1.dpad_left;
        rt = gamepad1.dpad_right;

        powerFlywheel(targetSpeed);

        prepareTelemetry();
        telemetry.update();
    }

    void prepareTelemetry(){
        if(setting_kP){
            telemetry.addData("Setting", "kP");
        }
        else telemetry.addData("Setting", "kF");
        telemetry.addData("Increment", presets[presetIndex%presets.length]);
        telemetry.addData("kF", kF);
        telemetry.addData("kP", kP);
        telemetry.addData("target speed", targetSpeed);
        telemetry.addData("motor speed", launcher.getVelocity());
        telemetry.addData("motor power", launcher.getPower());
        telemetry.addData("motor direction", launcher.getDirection());

    }

    void powerFlywheel(double targetSpeed){
        if(targetSpeed == 0){
            launcher.setPower(kP*(targetSpeed-launcher.getVelocity()));
        }
        else launcher.setPower(kF+kP*(targetSpeed-launcher.getVelocity()));
    }
}
