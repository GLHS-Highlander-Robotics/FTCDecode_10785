package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_Profile{

    double max_velo;
    public PID pid;
    public double target;
    public boolean isBusy;

    public PID_Profile(double p, double i, double d, double max_velo){
        pid = new PID(p, i, d);
        this.max_velo = max_velo;
    }

    public PID_Profile(double p, double i, double d, double f, double max_velo){
        pid = new PID(p, i, d, f);
        this.max_velo = max_velo;
    }

    public PID_Profile(PID pid1, double max_velo){
        pid = pid1;
        this.max_velo = max_velo;
    }

    public void setTarget(double input){
        target = input;
    }

    public void powerMotor(ElapsedTime elapsedTime, DcMotorEx motor, double input){
        double target_pos;

        target_pos = input+Math.signum(target-input)*max_velo*(elapsedTime.seconds()-pid.lastTime);

        if(input>target){
            target_pos = Math.max(target_pos, target);
        }
        else target_pos = Math.min(target_pos, target);

        motor.setPower(pid.output(elapsedTime,input, target_pos));
        isBusy = pid.isBusy;
    }

    public double output(ElapsedTime elapsedTime, double input){
        double target_pos;

        target_pos = input+Math.signum(target-input)*max_velo*(elapsedTime.seconds()-pid.lastTime);

        if(input>target){
            target_pos = Math.max(target_pos, target);
        }
        else target_pos = Math.min(target_pos, target);

        return pid.output(elapsedTime,input, target_pos);

    }

}
